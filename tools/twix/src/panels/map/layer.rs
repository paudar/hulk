use std::{marker::PhantomData, sync::Arc};

use color_eyre::Result;
use convert_case::{Case, Casing};
use eframe::egui::Ui;
use log::error;
use serde_json::{Value, json};

use types::field_dimensions::FieldDimensions;

use crate::{backend::TopicListState, robot::Robot, twix_painter::TwixPainter};

pub trait Layer<Frame> {
    const NAME: &'static str;
    fn new(robot: Arc<Robot>) -> Self;
    fn new_with_config(robot: Arc<Robot>, _value: Option<&Value>) -> Self
    where
        Self: Sized,
    {
        Self::new(robot)
    }
    fn config_ui(&mut self, _ui: &mut Ui) {}
    fn update_topics_from_discovery(&mut self, _topics: &TopicListState, _endpoint: &str) {}
    fn paint(&self, painter: &TwixPainter<Frame>, field_dimensions: &FieldDimensions)
    -> Result<()>;
    fn save_config(&self) -> Value {
        json!({})
    }
}

pub struct EnabledLayer<T, Frame>
where
    T: Layer<Frame>,
{
    robot: Arc<Robot>,
    layer: Option<T>,
    frame: PhantomData<Frame>,
}

impl<T, Frame> EnabledLayer<T, Frame>
where
    T: Layer<Frame>,
{
    pub fn new(robot: Arc<Robot>, value: Option<&Value>, active: bool) -> Self {
        let layer_value = value.and_then(|value| value.get(T::NAME.to_case(Case::Snake)));
        let active = layer_value
            .and_then(|value| value.get("active"))
            .and_then(|value| value.as_bool())
            .unwrap_or(active);
        let layer = active.then(|| T::new_with_config(robot.clone(), layer_value));
        Self {
            robot,
            layer,
            frame: PhantomData,
        }
    }

    pub fn checkbox(&mut self, ui: &mut Ui) {
        let mut active = self.layer.is_some();
        if ui.checkbox(&mut active, T::NAME).changed() {
            match self.layer.is_some() {
                false => self.layer = Some(T::new(self.robot.clone())),
                true => self.layer = None,
            }
        }
        if let Some(layer) = self.layer.as_mut() {
            layer.config_ui(ui);
        }
    }

    pub fn update_topics_from_discovery(&mut self, topics: &TopicListState, endpoint: &str) {
        if let Some(layer) = self.layer.as_mut() {
            layer.update_topics_from_discovery(topics, endpoint);
        }
    }

    pub fn paint_or_disable(
        &mut self,
        painter: &TwixPainter<Frame>,
        field_dimensions: &FieldDimensions,
    ) {
        if let Some(layer) = &self.layer
            && let Err(error) = layer.paint(painter, field_dimensions)
        {
            error!(
                "map panel: failed to paint map overlay {}: {:#}",
                T::NAME,
                error
            );
            self.layer = None;
        }
    }

    pub fn save(&self) -> Value {
        let mut value = json!({
            "active": self.layer.is_some(),
        });
        if let Some(layer) = &self.layer
            && let Value::Object(layer_config) = layer.save_config()
            && let Some(value) = value.as_object_mut()
        {
            value.extend(layer_config);
        }
        value
    }
}
