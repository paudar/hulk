use std::sync::Arc;

use color_eyre::{eyre::eyre, Result};
use coordinate_systems::Pixel;
use eframe::egui::{
    emath, epaint::PathShape, pos2, Color32, ColorImage, Pos2, Rect, Response, Sense, Shape,
    Stroke, TextureOptions, Ui, UiBuilder, Vec2, Widget,
};
use geometry::rectangle::Rectangle;
use image::RgbImage;
use linear_algebra::{point, vector};
use nalgebra::Vector3;
use serde_json::Value;

use types::{jpeg::JpegImage, ycbcr422_image::YCbCr422Image};

use crate::{
    nao::Nao,
    panel::Panel,
    twix_painter::{Orientation, TwixPainter},
    value_buffer::BufferHandle,
};

use super::image::cycler_selector::{VisionCycler, VisionCyclerSelector};

enum RawOrJpeg {
    Raw(BufferHandle<YCbCr422Image>),
    Jpeg(BufferHandle<JpegImage>),
}

pub struct ScribbleCalibrationPanel {
    nao: Arc<Nao>,
    _top_camera: BufferHandle<Vector3<f32>>,
    _bottom_camera: BufferHandle<Vector3<f32>>,

    image_buffer: RawOrJpeg,
    cycler: VisionCycler,

    line_1_goal: bool,
    line_2_penalty_horizontal: bool,
    line_3_penalty_left: bool,
    line_4_penalty_right: bool,

    lines: [(Pos2, Pos2); 4],
    stroke: Stroke,
}

impl Panel for ScribbleCalibrationPanel {
    const NAME: &'static str = "Scribble Calibration";

    fn new(nao: Arc<Nao>, value: Option<&Value>) -> Self {
        let _top_camera = nao.subscribe_value(
            "parameters.camera_matrix_parameters.vision_top.extrinsic_rotations".to_string(),
        );
        let _bottom_camera = nao.subscribe_value(
            "parameters.camera_matrix_parameters.vision_bottom.extrinsic_rotations".to_string(),
        );

        let cycler = value
            .and_then(|value| {
                let string = value.get("cycler")?.as_str()?;
                VisionCycler::try_from(string).ok()
            })
            .unwrap_or(VisionCycler::Top);
        let cycler_path = cycler.as_path();

        let is_jpeg = value
            .and_then(|value| value.get("is_jpeg"))
            .and_then(|value| value.as_bool())
            .unwrap_or(false);

        let image_buffer = if is_jpeg {
            let path = format!("{cycler_path}.main_outputs.image.jpeg");
            RawOrJpeg::Jpeg(nao.subscribe_value(path))
        } else {
            let path = format!("{cycler_path}.main_outputs.image");
            RawOrJpeg::Raw(nao.subscribe_value(path))
        };

        Self {
            nao,
            _top_camera,
            _bottom_camera,

            image_buffer,
            cycler,

            line_1_goal: true,
            line_2_penalty_horizontal: true,
            line_3_penalty_left: true,
            line_4_penalty_right: true,

            lines: [
                (pos2(50.0, 50.0), pos2(250.0, 50.0)),
                (pos2(50.0, 200.0), pos2(250.0, 200.0)),
                (pos2(110.0, 100.0), pos2(100.0, 150.0)),
                (pos2(190.0, 100.0), pos2(200.0, 150.0)),
            ],
            stroke: Stroke::new(1.0, Color32::from_rgb(25, 200, 100)),
        }
    }
}

impl Widget for &mut ScribbleCalibrationPanel {
    fn ui(self, ui: &mut Ui) -> Response {
        let jpeg = matches!(self.image_buffer, RawOrJpeg::Jpeg(_));
        let mut cycler_selector = VisionCyclerSelector::new(&mut self.cycler);
        if cycler_selector.ui(ui).changed() {
            self.resubscribe(jpeg);
        }
        ui.vertical(|ui| {
            ui.collapsing("Lines", |ui| {
                ui.vertical(|ui| {
                    ui.checkbox(&mut self.line_1_goal, "Line 1: Goal line");
                    ui.checkbox(
                        &mut self.line_2_penalty_horizontal,
                        "Line 2: Penalty horizontal",
                    );
                    ui.checkbox(&mut self.line_3_penalty_left, "Line 3: Penalty left");
                    ui.checkbox(&mut self.line_4_penalty_right, "Line 4: Penalty right");
                });
            });

            ui.separator();

            self.ui_content(ui);
        })
        .response
    }
}

impl ScribbleCalibrationPanel {
    fn int_to_bool(&self, number: usize) -> bool {
        match number {
            0 => self.line_1_goal,
            1 => self.line_2_penalty_horizontal,
            2 => self.line_3_penalty_left,
            3 => self.line_4_penalty_right,
            _ => false,
        }
    }

    // TODO: implement this only once
    fn resubscribe(&mut self, jpeg: bool) {
        let cycler_path = self.cycler.as_path();
        self.image_buffer = if jpeg {
            RawOrJpeg::Jpeg(
                self.nao
                    .subscribe_value(format!("{cycler_path}.main_outputs.image.jpeg")),
            )
        } else {
            RawOrJpeg::Raw(
                self.nao
                    .subscribe_value(format!("{cycler_path}.main_outputs.image")),
            )
        };
    }

    fn show_image(&self, painter: &TwixPainter<Pixel>) -> Result<Rect> {
        let context = painter.context();

        let image_identifier = format!("bytes://image-{:?}", self.cycler);
        let (image, width, height) = match &self.image_buffer {
            RawOrJpeg::Raw(buffer) => {
                let ycbcr = buffer
                    .get_last_value()?
                    .ok_or_else(|| eyre!("no image available"))?;
                let width = ycbcr.width() as usize;
                let height = ycbcr.height() as usize;
                let image = ColorImage::from_rgb([width, height], RgbImage::from(ycbcr).as_raw());
                (image, width, height)
            }
            RawOrJpeg::Jpeg(buffer) => {
                let jpeg = buffer
                    .get_last_value()?
                    .ok_or_else(|| eyre!("no image available"))?;
                let width = jpeg.width()? as usize;
                let height = jpeg.height()? as usize;
                let image = ColorImage::from_rgb([width, height], jpeg.data.as_slice());
                (image, width, height)
            }
        };

        let texture_id = context.load_texture(
            &image_identifier,
            image,
            TextureOptions::NEAREST,
        );

        let image_rect = Rect {
            min: pos2(0.0, 0.0),
            max: pos2(width as f32, height as f32),
        };

        painter.image(
            texture_id.id(),
            Rectangle {
                min: point!(0.0, 0.0),
                max: point!(640.0, 480.0),
            },
        );
        Ok(image_rect)
    }

    pub fn ui_content(&mut self, ui: &mut Ui) -> Response {
        let (response, mut painter) = TwixPainter::allocate(
            ui,
            vector![640.0, 480.0],
            point![0.0, 0.0],
            Orientation::LeftHanded,
        );

        let image_rect = match self.show_image(&painter) {
            Ok(rect) => rect,
            Err(error) => {
                ui.scope_builder(UiBuilder::new().max_rect(response.rect), |ui| {
                    ui.label(format!("{error}"))
                });
                return response;
            }
        };

        if let Err(error) = self.show_image(&painter) {
            ui.scope_builder(UiBuilder::new().max_rect(response.rect), |ui| {
                ui.label(format!("{error}"))
            });
        };

        let to_screen = emath::RectTransform::from_to(
            Rect::from_min_size(Pos2::ZERO, image_rect.size()),
            image_rect,
        );

        let line_states: Vec<bool> = (0..self.lines.len()).map(|i| self.int_to_bool(i)).collect();

        self.lines
            .iter_mut()
            .enumerate()
            .for_each(|(i, (start, end))| {
                if line_states[i] {
                    let start_screen = to_screen * *start;
                    let end_screen = to_screen * *end;

                    let radius = 5.0;
                    let start_response = ui.interact(
                        Rect::from_center_size(start_screen, Vec2::splat(radius * 2.0)),
                        response.id.with(format!("start_{}", i)),
                        Sense::drag(),
                    );
                    if start_response.dragged() {
                        *start += start_response.drag_delta() / to_screen.scale();
                        *start = to_screen.from().clamp(*start);
                    }
                    painter.add(Shape::circle_filled(start_screen, radius, Color32::WHITE));

                    let end_response = ui.interact(
                        Rect::from_center_size(end_screen, Vec2::splat(radius * 2.0)),
                        response.id.with(format!("end_{}", i)),
                        Sense::drag(),
                    );
                    if end_response.dragged() {
                        *end += end_response.drag_delta() / to_screen.scale();
                        *end = to_screen.from().clamp(*end);
                    }
                    painter.add(Shape::circle_filled(end_screen, radius, Color32::WHITE));

                    // Update the line with the new positions
                    let updated_start_screen = to_screen * *start;
                    let updated_end_screen = to_screen * *end;
                    painter.add(Shape::Path(PathShape::line(
                        vec![updated_start_screen, updated_end_screen],
                        self.stroke,
                    )));
                }
            });

        response
    }
}
