use core::fmt;

use serde::de::{self, DeserializeSeed, MapAccess, SeqAccess, Visitor};

use crate::trajectory_reading::{Action, Pose, Trajectory};

struct RootSeed<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> DeserializeSeed<'de> for RootSeed<'_> {
    type Value = bool;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        deserializer.deserialize_map(RootVisitor {
            trajectory: self.trajectory,
        })
    }
}

struct RootVisitor<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> Visitor<'de> for RootVisitor<'_> {
    type Value = bool;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a trajectory object")
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut result_seen = false;
        let mut has_record = false;

        while let Some(key) = map.next_key::<&str>()? {
            match key {
                "result" => {
                    if result_seen {
                        return Err(de::Error::custom("duplicate result field"));
                    }
                    result_seen = true;
                    has_record = map.next_value_seed(RecordListSeed {
                        trajectory: self.trajectory,
                    })?;
                }
                _ => {
                    map.next_value::<de::IgnoredAny>()?;
                }
            }
        }

        Ok(has_record)
    }
}

struct RecordListSeed<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> DeserializeSeed<'de> for RecordListSeed<'_> {
    type Value = bool;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        deserializer.deserialize_seq(RecordListVisitor {
            trajectory: self.trajectory,
        })
    }
}

struct RecordListVisitor<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> Visitor<'de> for RecordListVisitor<'_> {
    type Value = bool;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("an array containing one trajectory record")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        if seq
            .next_element_seed(RecordSeed {
                trajectory: self.trajectory,
            })?
            .is_none()
        {
            return Ok(false);
        }

        if seq.next_element::<de::IgnoredAny>()?.is_some() {
            return Err(de::Error::custom(
                "trajectory result must contain exactly one record",
            ));
        }

        Ok(true)
    }
}

struct RecordSeed<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> DeserializeSeed<'de> for RecordSeed<'_> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        deserializer.deserialize_map(RecordVisitor {
            trajectory: self.trajectory,
        })
    }
}

struct RecordVisitor<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> Visitor<'de> for RecordVisitor<'_> {
    type Value = ();

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a trajectory record")
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut states_seen = false;
        let mut actions_seen = false;

        while let Some(key) = map.next_key::<&str>()? {
            match key {
                "states" => {
                    if states_seen {
                        return Err(de::Error::custom("duplicate states field"));
                    }
                    states_seen = true;
                    map.next_value_seed(StatesSeed {
                        trajectory: self.trajectory,
                    })?;
                }
                "actions" => {
                    if actions_seen {
                        return Err(de::Error::custom("duplicate actions field"));
                    }
                    actions_seen = true;
                    map.next_value_seed(ActionsSeed {
                        trajectory: self.trajectory,
                    })?;
                }
                _ => {
                    map.next_value::<de::IgnoredAny>()?;
                }
            }
        }

        Ok(())
    }
}

struct StatesSeed<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> DeserializeSeed<'de> for StatesSeed<'_> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        deserializer.deserialize_seq(StatesVisitor {
            trajectory: self.trajectory,
        })
    }
}

struct StatesVisitor<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> Visitor<'de> for StatesVisitor<'_> {
    type Value = ();

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("an array of three-number states")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        while let Some([x, y, yaw]) = seq.next_element::<[f32; 3]>()? {
            self.trajectory
                .states
                .push(Pose { x, y, yaw })
                .map_err(|_| de::Error::custom("states exceed capacity"))?;
        }

        Ok(())
    }
}

struct ActionsSeed<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> DeserializeSeed<'de> for ActionsSeed<'_> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: de::Deserializer<'de>,
    {
        deserializer.deserialize_seq(ActionsVisitor {
            trajectory: self.trajectory,
        })
    }
}

struct ActionsVisitor<'a> {
    trajectory: &'a mut Trajectory,
}

impl<'de> Visitor<'de> for ActionsVisitor<'_> {
    type Value = ();

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("an array of two-number actions")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        while let Some([v, omega]) = seq.next_element::<[f32; 2]>()? {
            self.trajectory
                .actions
                .push(Action { v, omega })
                .map_err(|_| de::Error::custom("actions exceed capacity"))?;
        }

        Ok(())
    }
}

pub fn from_json(mut json: &[u8]) -> Result<Trajectory, &'static str> {
    if json.len() >= 3 && &json[..3] == b"\xEF\xBB\xBF" {
        json = &json[3..];
    }

    let mut trajectory = Trajectory::default();
    let mut deserializer = serde_json_core::de::Deserializer::new(json);
    let has_record = RootSeed {
        trajectory: &mut trajectory,
    }
    .deserialize(&mut deserializer)
    .map_err(|e| {
        defmt::warn!("serde err: {:?}", defmt::Debug2Format(&e));
        "JSON parse failed"
    })?;
    deserializer.end().map_err(|e| {
        defmt::warn!("serde err: {:?}", defmt::Debug2Format(&e));
        "JSON parse failed"
    })?;

    if !has_record {
        return Err("result empty");
    }
    if trajectory.states.is_empty() || trajectory.actions.is_empty() {
        return Err("empty states/actions");
    }

    Ok(trajectory)
}
