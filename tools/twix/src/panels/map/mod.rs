use std::sync::Arc;

use coordinate_systems::{Field, Ground};
use eframe::egui::{ComboBox, Ui, Widget};
use linear_algebra::{Isometry2, point, vector};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use types::field_dimensions::FieldDimensions;

use crate::{
    backend::TopicListState,
    panel::{Panel, PanelCreationContext},
    robot::Robot,
    topic_completion_edit::TopicCompletionEdit,
    twix_painter::{Orientation, TwixPainter},
    value_buffer::BufferHandle,
    zoom_and_pan::ZoomAndPanTransform,
};

use self::layer::{EnabledLayer, Layer};

mod layer;
mod layers;

const FIELD_DIMENSIONS_TOPIC_NAME: &str = "field_dimensions";
const GROUND_TO_FIELD_TOPIC_NAME: &str = "ground_to_field";
const LINE_DATA_TOPIC_NAME: &str = "line_data";
const DEFAULT_FIELD_DIMENSIONS_TOPIC: &str = "/field_dimensions";
const DEFAULT_GROUND_TO_FIELD_TOPIC: &str = "/ground_to_field";
const DEFAULT_LINE_DATA_TOPIC: &str = "/line_data";

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
enum PlotType {
    Field,
    Ground,
}

trait GenericLayer {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    );
}

impl<T: Layer<Field>> GenericLayer for EnabledLayer<T, Field> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        _ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint_or_disable(painter, field_dimensions)
    }
}

impl<T: Layer<Ground>> GenericLayer for EnabledLayer<T, Ground> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint_or_disable(
            &painter.transform_painter(ground_to_field.inverse()),
            field_dimensions,
        )
    }
}

struct JsonTopicSubscription {
    topic: String,
    buffer: BufferHandle<Value>,
    qos: JsonTopicQos,
}

impl JsonTopicSubscription {
    fn new(robot: &Robot, topic: String, qos: JsonTopicQos) -> Self {
        let buffer = qos.subscribe(robot, topic.clone());
        Self { topic, buffer, qos }
    }

    fn resubscribe(&mut self, robot: &Robot) {
        self.buffer = self.qos.subscribe(robot, self.topic.clone());
    }

    fn update_from_discovery(
        &mut self,
        robot: &Robot,
        topic_name: &str,
        topics: &TopicListState,
        endpoint: &str,
    ) {
        let Some(topic) = topic_from_discovery(&self.topic, topic_name, topics, endpoint) else {
            return;
        };
        self.topic = topic;
        self.resubscribe(robot);
    }
}

#[derive(Clone, Copy)]
enum JsonTopicQos {
    Volatile,
    TransientLocal,
}

impl JsonTopicQos {
    fn subscribe(self, robot: &Robot, topic: String) -> BufferHandle<Value> {
        match self {
            Self::Volatile => robot.subscribe_json(topic),
            Self::TransientLocal => robot.subscribe_transient_local_json(topic),
        }
    }
}

pub struct MapPanel {
    current_plot_type: PlotType,

    robot: Arc<Robot>,
    field_dimensions: JsonTopicSubscription,
    ground_to_field: JsonTopicSubscription,
    zoom_and_pan: ZoomAndPanTransform,

    field: EnabledLayer<layers::Field, Field>,
    image_segments: EnabledLayer<layers::ImageSegments, Ground>,
    lines: EnabledLayer<layers::Lines, Ground>,
    ball_search_heatmap: EnabledLayer<layers::BallSearchHeatmap, Field>,
    line_correspondences: EnabledLayer<layers::LineCorrespondences, Field>,
    path_obstacles: EnabledLayer<layers::PathObstacles, Ground>,
    obstacles: EnabledLayer<layers::Obstacles, Ground>,
    path: EnabledLayer<layers::Path, Ground>,
    behavior_simulator: EnabledLayer<layers::BehaviorSimulator, Field>,
    robot_pose: EnabledLayer<layers::RobotPose, Ground>,
    referee_position: EnabledLayer<layers::RefereePosition, Field>,
    pose_detection: EnabledLayer<layers::PoseDetection, Field>,
    ball_percept: EnabledLayer<layers::BallPercepts, Ground>,
    ball_position: EnabledLayer<layers::BallPosition, Field>,
    ball_filter: EnabledLayer<layers::BallFilter, Ground>,
    obstacle_filter: EnabledLayer<layers::ObstacleFilter, Ground>,
    localization: EnabledLayer<layers::Localization, Field>,
}

impl<'a> Panel<'a> for MapPanel {
    const NAME: &'static str = "Map";

    fn new(context: PanelCreationContext) -> Self {
        let field = EnabledLayer::new(context.robot.clone(), context.value, true);
        let image_segments = EnabledLayer::new(context.robot.clone(), context.value, false);
        let line_correspondences = EnabledLayer::new(context.robot.clone(), context.value, false);
        let lines = EnabledLayer::new(context.robot.clone(), context.value, true);
        let ball_search_heatmap = EnabledLayer::new(context.robot.clone(), context.value, false);
        let path_obstacles = EnabledLayer::new(context.robot.clone(), context.value, false);
        let obstacles = EnabledLayer::new(context.robot.clone(), context.value, false);
        let path = EnabledLayer::new(context.robot.clone(), context.value, false);
        let behavior_simulator = EnabledLayer::new(context.robot.clone(), context.value, false);
        let referee_position = EnabledLayer::new(context.robot.clone(), context.value, false);
        let robot_pose = EnabledLayer::new(context.robot.clone(), context.value, true);
        let ball_percept = EnabledLayer::new(context.robot.clone(), context.value, false);
        let pose_detection = EnabledLayer::new(context.robot.clone(), context.value, false);
        let ball_position = EnabledLayer::new(context.robot.clone(), context.value, true);
        let ball_filter = EnabledLayer::new(context.robot.clone(), context.value, false);
        let obstacle_filter = EnabledLayer::new(context.robot.clone(), context.value, false);
        let localization = EnabledLayer::new(context.robot.clone(), context.value, false);

        let field_dimensions_topic = context
            .value
            .and_then(|value| value.get("field_dimensions_topic"))
            .and_then(Value::as_str)
            .map(ToString::to_string)
            .unwrap_or_else(|| DEFAULT_FIELD_DIMENSIONS_TOPIC.to_string());
        let field_dimensions = JsonTopicSubscription::new(
            &context.robot,
            field_dimensions_topic,
            JsonTopicQos::TransientLocal,
        );
        let ground_to_field_topic = context
            .value
            .and_then(|value| value.get("ground_to_field_topic"))
            .and_then(Value::as_str)
            .map(ToString::to_string)
            .unwrap_or_else(|| DEFAULT_GROUND_TO_FIELD_TOPIC.to_string());
        let ground_to_field = JsonTopicSubscription::new(
            &context.robot,
            ground_to_field_topic,
            JsonTopicQos::Volatile,
        );

        let current_plot_type = context
            .value
            .and_then(|value| value.get("current_plot_type"))
            .and_then(|value| serde_json::from_value::<PlotType>(value.clone()).ok())
            .unwrap_or(PlotType::Ground);
        let zoom_and_pan = context
            .value
            .and_then(|value| value.get("zoom_and_pan"))
            .and_then(|value| serde_json::from_value::<ZoomAndPanTransform>(value.clone()).ok())
            .unwrap_or_default();

        Self {
            current_plot_type,
            robot: context.robot,
            field_dimensions,
            ground_to_field,
            zoom_and_pan,
            field,
            image_segments,
            line_correspondences,
            lines,
            ball_search_heatmap,
            path_obstacles,
            obstacles,
            path,
            behavior_simulator,
            robot_pose,
            pose_detection,
            referee_position,
            ball_percept,
            ball_position,
            ball_filter,
            obstacle_filter,
            localization,
        }
    }

    fn save(&self) -> Value {
        json!({
            "current_plot_type": self.current_plot_type,
            "zoom_and_pan": serde_json::to_value(&self.zoom_and_pan).expect("failed to serialize zoom_and_pan"),
            "field_dimensions_topic": self.field_dimensions.topic,
            "ground_to_field_topic": self.ground_to_field.topic,

            "field": self.field.save(),
            "image_segments": self.image_segments.save(),
            "line_correspondences": self.line_correspondences.save(),
            "lines": self.lines.save(),
            "ball_search_heatmap": self.obstacle_filter.save(),
            "path_obstacles": self.path_obstacles.save(),
            "obstacles": self.obstacles.save(),
            "path": self.path.save(),
            "behavior_simulator": self.behavior_simulator.save(),
            "pose_detection": self.referee_position.save(),
            "robot_pose": self.robot_pose.save(),
            "referee_position": self.referee_position.save(),
            "ball_percept": self.ball_percept.save(),
            "ball_position": self.ball_position.save(),
            "ball_filter": self.ball_filter.save(),
            "obstacle_filter": self.obstacle_filter.save(),
            "localization": self.localization.save(),
        })
    }
}

impl Widget for &mut MapPanel {
    fn ui(self, ui: &mut Ui) -> eframe::egui::Response {
        let topics = self.robot.topic_list_state();
        self.update_topics_from_discovery(&topics);

        ui.horizontal(|ui| {
            ui.menu_button("Overlays", |ui| {
                self.field.checkbox(ui);
                self.image_segments.checkbox(ui);
                self.line_correspondences.checkbox(ui);
                self.lines.checkbox(ui);
                self.ball_search_heatmap.checkbox(ui);
                self.path_obstacles.checkbox(ui);
                self.obstacles.checkbox(ui);
                self.path.checkbox(ui);
                self.behavior_simulator.checkbox(ui);
                self.pose_detection.checkbox(ui);
                self.robot_pose.checkbox(ui);
                self.referee_position.checkbox(ui);
                self.ball_percept.checkbox(ui);
                self.ball_position.checkbox(ui);
                self.ball_filter.checkbox(ui);
                self.obstacle_filter.checkbox(ui);
                self.localization.checkbox(ui);
            });
            ComboBox::from_id_salt("plot_type_selector")
                .selected_text(format!("{:?}", self.current_plot_type))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.current_plot_type, PlotType::Ground, "Ground");
                    ui.selectable_value(&mut self.current_plot_type, PlotType::Field, "Field");
                });
        });

        ui.horizontal(|ui| {
            ui.label("Field dimensions");
            let response = ui.add(TopicCompletionEdit::new(
                ui.id().with("field_dimensions_topic"),
                &topics,
                &mut self.field_dimensions.topic,
            ));
            if response.changed() {
                self.field_dimensions.resubscribe(&self.robot);
            }
        });

        ui.horizontal(|ui| {
            ui.label("Ground to field");
            let response = ui.add(TopicCompletionEdit::new(
                ui.id().with("ground_to_field_topic"),
                &topics,
                &mut self.ground_to_field.topic,
            ));
            if response.changed() {
                self.ground_to_field.resubscribe(&self.robot);
            }
        });

        let field_dimensions = match self.field_dimensions.buffer.get_last_value() {
            Ok(Some(value)) => match field_dimensions_from_json(&value) {
                Some(field_dimensions) => field_dimensions,
                None => return ui.label("failed to parse field dimensions JSON"),
            },
            Ok(None) => {
                return ui.label(format!(
                    "no response for field dimensions on {}",
                    self.field_dimensions.topic
                ));
            }
            Err(error) => return ui.label(format!("{error:#}")),
        };

        let ground_to_field = self
            .ground_to_field
            .buffer
            .get_last_value()
            .ok()
            .flatten()
            .and_then(|value| ground_to_field_from_json(&value))
            .unwrap_or_default();
        let (response, mut painter) = match self.current_plot_type {
            PlotType::Field => {
                let width = field_dimensions.width;
                let length = field_dimensions.length;
                let border = field_dimensions.border_strip_width;

                TwixPainter::allocate(
                    ui,
                    vector![2.0 * border + length, 2.0 * border + width],
                    point![
                        border + field_dimensions.length / 2.0,
                        -border - field_dimensions.width / 2.0
                    ],
                    Orientation::RightHanded,
                )
            }
            PlotType::Ground => {
                let (response, painter) = TwixPainter::allocate(
                    ui,
                    vector![2.0, 2.0],
                    point![1.0, -1.0],
                    Orientation::RightHanded,
                );
                (response, painter.transform_painter(ground_to_field))
            }
        };
        self.zoom_and_pan.apply(ui, &mut painter, &response);

        // draw largest layers first so they don't obscure smaller ones
        self.field
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.image_segments
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        self.line_correspondences
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.lines
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        self.ball_search_heatmap
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.path_obstacles
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.obstacles
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.path
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.behavior_simulator
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.robot_pose
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.referee_position
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_percept
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_position
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.pose_detection
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_position
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_filter
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.obstacle_filter
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.localization
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        response
    }
}

impl MapPanel {
    fn update_topics_from_discovery(&mut self, topics: &TopicListState) {
        let endpoint = self.robot.endpoint();
        self.field_dimensions.update_from_discovery(
            &self.robot,
            FIELD_DIMENSIONS_TOPIC_NAME,
            topics,
            &endpoint,
        );
        self.ground_to_field.update_from_discovery(
            &self.robot,
            GROUND_TO_FIELD_TOPIC_NAME,
            topics,
            &endpoint,
        );
        self.lines.update_topics_from_discovery(topics, &endpoint);
    }
}

fn topic_from_discovery(
    current_topic: &str,
    topic_name: &str,
    topics: &TopicListState,
    endpoint: &str,
) -> Option<String> {
    if topic_basename(current_topic) != Some(topic_name) {
        return None;
    }

    if let Some(robot_number) = robot_number_from_endpoint(endpoint) {
        let robot_topic = format!("/{robot_number}/{topic_name}");
        if topic_exists(topics, &robot_topic) && current_topic != robot_topic {
            return Some(robot_topic);
        }
    }

    if topic_exists(topics, current_topic) {
        return None;
    }

    let root_topic = format!("/{topic_name}");
    if topic_exists(topics, &root_topic) {
        return Some(root_topic);
    }

    let candidates = topics
        .topics
        .iter()
        .filter(|topic| topic_basename(&topic.name) == Some(topic_name))
        .map(|topic| topic.name.as_str())
        .collect::<Vec<_>>();

    match candidates.as_slice() {
        [topic] if *topic != current_topic => Some((*topic).to_string()),
        _ => None,
    }
}

fn topic_exists(topics: &TopicListState, topic_name: &str) -> bool {
    topics.topics.iter().any(|topic| topic.name == topic_name)
}

fn topic_basename(topic_name: &str) -> Option<&str> {
    topic_name
        .trim_end_matches('/')
        .rsplit('/')
        .next()
        .filter(|name| !name.is_empty())
}

fn robot_number_from_endpoint(endpoint: &str) -> Option<String> {
    endpoint
        .split(|character: char| !character.is_ascii_digit() && character != '.')
        .find_map(|part| {
            let octets = part.split('.').collect::<Vec<_>>();
            let [first, second, third, fourth] = octets.as_slice() else {
                return None;
            };
            if *first != "10" || (*second != "0" && *second != "1") || *third != "24" {
                return None;
            }
            let number = fourth.parse::<u8>().ok()?;
            (number != 0 && number != 255).then(|| number.to_string())
        })
}

fn field_dimensions_from_json(value: &Value) -> Option<FieldDimensions> {
    parse_json_value_or_nested(value, |value| serde_json::from_value(value.clone()).ok())
}

fn ground_to_field_from_json(value: &Value) -> Option<Isometry2<Ground, Field>> {
    parse_json_value_or_nested(value, ground_to_field_from_json_value)
}

fn parse_json_value_or_nested<T>(
    value: &Value,
    parser: impl Fn(&Value) -> Option<T> + Copy,
) -> Option<T> {
    parser(value)
        .or_else(|| value.get("value").and_then(parser))
        .or_else(|| value.get("inner").and_then(parser))
        .or_else(|| value.get("payload").and_then(parser))
}

fn ground_to_field_from_json_value(value: &Value) -> Option<Isometry2<Ground, Field>> {
    if value.is_null() {
        return None;
    }

    serde_json::from_value(value.clone())
        .ok()
        .or_else(|| ground_to_field_from_nalgebra_json(value))
        .or_else(|| ground_to_field_from_pose_json(value))
}

fn ground_to_field_from_nalgebra_json(value: &Value) -> Option<Isometry2<Ground, Field>> {
    let translation = translation_from_json(value.get("translation")?)?;
    let rotation = angle_from_json(value.get("rotation")?)?;
    Some(Isometry2::from_parts(translation, rotation))
}

fn ground_to_field_from_pose_json(value: &Value) -> Option<Isometry2<Ground, Field>> {
    let translation = value
        .get("position")
        .or_else(|| value.get("translation"))
        .and_then(translation_from_json)
        .or_else(|| {
            let x = number_from_json(value.get("x")?)?;
            let y = number_from_json(value.get("y")?)?;
            Some(vector![x, y])
        })?;

    let rotation = value
        .get("orientation")
        .or_else(|| value.get("rotation"))
        .and_then(angle_from_json)
        .or_else(|| value.get("theta").and_then(number_from_json))
        .unwrap_or_default();

    Some(Isometry2::from_parts(translation, rotation))
}

fn translation_from_json(value: &Value) -> Option<linear_algebra::Vector2<Field>> {
    parse_json_value_or_nested(value, |value| {
        if let Some(values) = value.as_array() {
            return Some(vector![
                number_from_json(values.first()?)?,
                number_from_json(values.get(1)?)?
            ]);
        }

        let x = value
            .get("x")
            .or_else(|| value.get("X"))
            .and_then(number_from_json)?;
        let y = value
            .get("y")
            .or_else(|| value.get("Y"))
            .and_then(number_from_json)?;
        Some(vector![x, y])
    })
}

fn angle_from_json(value: &Value) -> Option<f32> {
    parse_json_value_or_nested(value, |value| {
        if let Some(angle) = value
            .get("angle")
            .or_else(|| value.get("radians"))
            .or_else(|| value.get("theta"))
            .and_then(number_from_json)
        {
            return Some(angle);
        }

        if let Some(values) = value.as_array() {
            let real = number_from_json(values.first()?)?;
            let imaginary = number_from_json(values.get(1)?)?;
            return Some(imaginary.atan2(real));
        }

        let real = value
            .get("re")
            .or_else(|| value.get("real"))
            .or_else(|| value.get("cos"))
            .and_then(number_from_json)?;
        let imaginary = value
            .get("im")
            .or_else(|| value.get("imaginary"))
            .or_else(|| value.get("sin"))
            .and_then(number_from_json)?;
        Some(imaginary.atan2(real))
    })
}

fn number_from_json(value: &Value) -> Option<f32> {
    value.as_f64().map(|value| value as f32)
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use crate::backend::TopicDescriptor;

    use super::*;

    #[test]
    fn parses_ground_to_field_from_ros_z_nalgebra_json() {
        let ground_to_field = ground_to_field_from_json(&json!({
            "translation": [1.5, -0.25],
            "rotation": [0.0, 1.0],
        }))
        .unwrap();

        assert_near(ground_to_field.translation().x(), 1.5);
        assert_near(ground_to_field.translation().y(), -0.25);
        assert_near(
            ground_to_field.orientation().angle(),
            std::f32::consts::FRAC_PI_2,
        );
    }

    #[test]
    fn parses_ground_to_field_from_wrapped_json() {
        let ground_to_field = ground_to_field_from_json(&json!({
            "value": {
                "translation": [2.0, 3.0],
                "rotation": [1.0, 0.0],
            }
        }))
        .unwrap();

        assert_near(ground_to_field.translation().x(), 2.0);
        assert_near(ground_to_field.translation().y(), 3.0);
        assert_near(ground_to_field.orientation().angle(), 0.0);
    }

    #[test]
    fn parses_ground_to_field_from_pose_like_json() {
        let ground_to_field = ground_to_field_from_json(&json!({
            "position": {
                "x": -1.0,
                "y": 0.5,
            },
            "orientation": {
                "angle": 0.25,
            },
        }))
        .unwrap();

        assert_near(ground_to_field.translation().x(), -1.0);
        assert_near(ground_to_field.translation().y(), 0.5);
        assert_near(ground_to_field.orientation().angle(), 0.25);
    }

    #[test]
    fn null_ground_to_field_means_no_pose() {
        assert!(ground_to_field_from_json(&Value::Null).is_none());
    }

    #[test]
    fn parses_field_dimensions_from_json() {
        let field_dimensions = field_dimensions_from_json(&json!({
            "ball_radius": 0.05,
            "length": 9.0,
            "width": 6.0,
            "line_width": 0.05,
            "penalty_marker_size": 0.1,
            "goal_box_area_length": 0.6,
            "goal_box_area_width": 2.2,
            "penalty_area_length": 1.65,
            "penalty_area_width": 4.0,
            "penalty_marker_distance": 1.3,
            "center_circle_diameter": 1.5,
            "border_strip_width": 2.0,
            "goal_inner_width": 1.5,
            "goal_post_diameter": 0.1,
            "goal_depth": 0.5,
            "corner_arc_radius": 0.0,
        }))
        .unwrap();

        assert_near(field_dimensions.length, 9.0);
        assert_near(field_dimensions.width, 6.0);
    }

    #[test]
    fn selects_unique_namespaced_topic_from_discovery() {
        let topics = topics(["/43/field_dimensions"]);

        assert_eq!(
            topic_from_discovery(
                "/field_dimensions",
                FIELD_DIMENSIONS_TOPIC_NAME,
                &topics,
                "tcp/127.0.0.1:7447"
            ),
            Some("/43/field_dimensions".to_string())
        );
    }

    #[test]
    fn prefers_endpoint_robot_namespace_when_topics_are_ambiguous() {
        let topics = topics(["/42/field_dimensions", "/43/field_dimensions"]);

        assert_eq!(
            topic_from_discovery(
                "/field_dimensions",
                FIELD_DIMENSIONS_TOPIC_NAME,
                &topics,
                "tcp/10.1.24.43:7447"
            ),
            Some("/43/field_dimensions".to_string())
        );
    }

    #[test]
    fn keeps_ambiguous_topic_without_endpoint_robot_number() {
        let topics = topics(["/42/field_dimensions", "/43/field_dimensions"]);

        assert_eq!(
            topic_from_discovery(
                "/field_dimensions",
                FIELD_DIMENSIONS_TOPIC_NAME,
                &topics,
                "tcp/127.0.0.1:7447"
            ),
            None
        );
    }

    #[test]
    fn extracts_robot_number_from_hulks_robot_endpoint() {
        assert_eq!(
            robot_number_from_endpoint("tcp/10.1.24.43:7447"),
            Some("43".to_string())
        );
        assert_eq!(
            robot_number_from_endpoint("tcp/10.0.24.21:7447"),
            Some("21".to_string())
        );
        assert_eq!(robot_number_from_endpoint("tcp/127.0.0.1:7447"), None);
    }

    fn assert_near(left: f32, right: f32) {
        assert!((left - right).abs() < 0.0001, "{left} != {right}");
    }

    fn topics<const COUNT: usize>(names: [&str; COUNT]) -> TopicListState {
        TopicListState {
            discovering: false,
            topics: names
                .into_iter()
                .map(|name| TopicDescriptor {
                    name: name.to_string(),
                    graph_type: String::new(),
                })
                .collect(),
        }
    }
}
