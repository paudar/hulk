use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use crate::field_dimensions::Side;

#[derive(
    Debug,
    Clone,
    Copy,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    Serialize,
    Deserialize,
    PartialEq,
    Eq,
)]
pub enum Action {
    Animation,
    Calibrate,
    DefendGoal,
    DefendKickOff,
    DefendLeft,
    DefendPenaltyKick,
    DefendRight,
    DefendOpponentCornerKick { side: Side },
    Dribble,
    FallSafely,
    Initial,
    InterceptBall,
    Jump,
    LookAround,
    LookAtReferee,
    NoGroundContact,
    Penalize,
    PrepareJump,
    Search,
    SearchForLostBall,
    SitDown,
    Stand,
    StandUp,
    KeeperMotion,
    SupportLeft,
    SupportRight,
    SupportStriker,
    Unstiff,
    WalkToKickOff,
    WalkToPenaltyKick,
}
