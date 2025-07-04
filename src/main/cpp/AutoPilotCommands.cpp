#include "AutoPilot.h"

//close_shot

void L4PlaceCommand::Initialize() {
    level_handle.try_take_control();
    level_handle.set(3);
    action_handle.try_take_control();
    action_handle.set(LiftMechanismState::place);
}

void L4PlaceCommand::Execute() {
    
}

void L4PlaceCommand::End(bool interrupted) {
    //if(interrupted) action_handle.set(LiftMechanismState::mid);
}

bool L4PlaceCommand::IsFinished() {
    if(lift->get_state() == LiftDetailedState::eject_coral) return true;
    else return false;
}

void LiftMidCommand::Initialize() {
    action_handle.try_take_control();
    action_handle.set(LiftMechanismState::mid);
}

void LiftMidCommand::Execute() {
    
}

void LiftMidCommand::End(bool interrupted) {
    
}

bool LiftMidCommand::IsFinished() {
    if(lift->get_state() == LiftDetailedState::mid) return true;
    else return false;
}

void UnmaskVisionCommand::Initialize() {
    mask_handle.try_take_control();
    mask_handle.set(false);
}

void UnmaskVisionCommand::Execute() {
    
}

void UnmaskVisionCommand::End(bool interrupted) {
    
}

bool UnmaskVisionCommand::IsFinished() {
    return !mask_handle.get();
}

void AutoAlignLeftCommand::Initialize() {
    tree_handle.try_take_control();
    tree_handle.set(ReefTree::right);

    twist_handle.try_take_control();
    twist_handle.set(AutoPilotTwistMode::reef);

    lateral_handle.try_take_control();
    lateral_handle.set(AutoPilotTranslateMode::reef);
}

void AutoAlignLeftCommand::Execute() {
    
}

void AutoAlignLeftCommand::End(bool interrupted) {
    
}

bool AutoAlignLeftCommand::IsFinished() {
    return auto_pilot->is_finished();
}