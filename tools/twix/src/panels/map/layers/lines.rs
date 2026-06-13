use std::sync::Arc;

use color_eyre::Result;
use eframe::egui::Ui;
use eframe::epaint::{Color32, Stroke};
use serde_json::{Value, json};

use coordinate_systems::Ground;
use geometry::line_segment::LineSegment;
use linear_algebra::{Point2, point};
use types::field_dimensions::FieldDimensions;
use types::line_data::LineData;

use crate::{
    panels::map::{
        DEFAULT_LINE_DATA_TOPIC, JsonTopicQos, JsonTopicSubscription, LINE_DATA_TOPIC_NAME,
        layer::Layer,
    },
    robot::Robot,
    topic_completion_edit::TopicCompletionEdit,
    twix_painter::TwixPainter,
};

pub struct Lines {
    robot: Arc<Robot>,
    line_data: JsonTopicSubscription,
}

impl Layer<Ground> for Lines {
    const NAME: &'static str = "Lines";

    fn new(robot: Arc<Robot>) -> Self {
        Self::new_with_topic(robot, DEFAULT_LINE_DATA_TOPIC.to_string())
    }

    fn new_with_config(robot: Arc<Robot>, value: Option<&Value>) -> Self {
        let topic = value
            .and_then(|value| value.get("topic"))
            .and_then(Value::as_str)
            .map(ToString::to_string)
            .unwrap_or_else(|| DEFAULT_LINE_DATA_TOPIC.to_string());
        Self::new_with_topic(robot, topic)
    }

    fn config_ui(&mut self, ui: &mut Ui) {
        let topics = self.robot.topic_list_state();
        ui.horizontal(|ui| {
            ui.label("Topic");
            let response = ui.add(TopicCompletionEdit::new(
                ui.id().with("line_data_topic"),
                &topics,
                &mut self.line_data.topic,
            ));
            if response.changed() {
                self.line_data.resubscribe(&self.robot);
            }
        });
    }

    fn update_topics_from_discovery(
        &mut self,
        topics: &crate::backend::TopicListState,
        endpoint: &str,
    ) {
        self.line_data
            .update_from_discovery(&self.robot, LINE_DATA_TOPIC_NAME, topics, endpoint);
    }

    fn save_config(&self) -> Value {
        json!({
            "topic": self.line_data.topic,
        })
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let Some(line_data) = self.line_data.buffer.get_last_value()? else {
            return Ok(());
        };
        let Some(lines) = line_data_lines_from_json(&line_data) else {
            return Ok(());
        };

        for line in lines {
            painter.line_segment(line.0, line.1, Stroke::new(0.04, Color32::RED));
        }

        Ok(())
    }
}

impl Lines {
    fn new_with_topic(robot: Arc<Robot>, topic: String) -> Self {
        let line_data = JsonTopicSubscription::new(&robot, topic, JsonTopicQos::Volatile);
        Self { robot, line_data }
    }
}

fn line_data_lines_from_json(value: &Value) -> Option<Vec<LineSegment<Ground>>> {
    parse_json_value_or_nested(value, |value| {
        serde_json::from_value::<LineData>(value.clone())
            .ok()
            .map(|line_data| line_data.lines)
            .or_else(|| {
                value
                    .get("lines")
                    .and_then(line_segments_from_json)
                    .or_else(|| {
                        value
                            .get("lines_in_ground")
                            .and_then(line_segments_from_json)
                    })
            })
    })
}

fn line_segments_from_json(value: &Value) -> Option<Vec<LineSegment<Ground>>> {
    value
        .as_array()?
        .iter()
        .map(line_segment_from_json)
        .collect()
}

fn line_segment_from_json(value: &Value) -> Option<LineSegment<Ground>> {
    serde_json::from_value::<LineSegment<Ground>>(value.clone())
        .ok()
        .or_else(|| {
            let values = value.as_array()?;
            Some(LineSegment(
                point_from_json(values.first()?)?,
                point_from_json(values.get(1)?)?,
            ))
        })
        .or_else(|| {
            Some(LineSegment(
                value
                    .get("start")
                    .or_else(|| value.get("from"))
                    .or_else(|| value.get("0"))
                    .and_then(point_from_json)?,
                value
                    .get("end")
                    .or_else(|| value.get("to"))
                    .or_else(|| value.get("1"))
                    .and_then(point_from_json)?,
            ))
        })
}

fn point_from_json(value: &Value) -> Option<Point2<Ground>> {
    parse_json_value_or_nested(value, |value| {
        if let Some(values) = value.as_array() {
            return Some(point![
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
        Some(point![x, y])
    })
}

fn parse_json_value_or_nested<T>(
    value: &Value,
    parser: impl Fn(&Value) -> Option<T> + Copy,
) -> Option<T> {
    parser(value)
        .or_else(|| value.get("value").and_then(parser))
        .or_else(|| value.get("inner").and_then(parser))
        .or_else(|| value.get("payload").and_then(parser))
        .or_else(|| value.get("coords").and_then(parser))
}

fn number_from_json(value: &Value) -> Option<f32> {
    value.as_f64().map(|value| value as f32)
}
