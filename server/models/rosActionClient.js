import * as ROSLIB from 'roslib';

export class ROS2GoalActionClient {
  constructor(ros, actionName, actionType, feedback = true) {
    this.ros = ros;
    this.actionName = actionName;
    this.actionType = actionType;
    this.goalCounter = 0;
    this.activeGoals = new Map();
    this.feedback = feedback || true;
  }
  
  sendGoal(args, callbacks = {}) {
    const goalId = `${this.actionName}_goal_${++this.goalCounter}_${Date.now()}`;
    
    const goalInfo = {
      id: goalId,
      callbacks: callbacks
    };
    
  const messageListener = (message) => {
    // Aquí diferencias por el campo 'op' del mensaje
    if (message.op === 'action_feedback') {
      // Es feedback
      if (callbacks.onFeedback) {
        callbacks.onFeedback(message.values);
      }
    } 
    else if (message.op === 'action_result') {
      // Es resultado final
      if (callbacks.onResult) {
        callbacks.onResult({
          result: message.result,
          status: message.status,
          values: message.values
        });
      }
      
      // Limpiar cuando termine
      this.ros.off(goalId, messageListener);
      this.activeGoals.delete(goalId);
    }
  };
  
  // Registrar el listener
  this.ros.on(goalId, messageListener);
    this.activeGoals.set(goalId, goalInfo);
    
    // Enviar goal usando callOnConnection
    const message = {
      op: 'send_action_goal',
      action: this.actionName,
      action_type: this.actionType,
      args: args,
      feedback: this.feedback,
      id: goalId
    };
    
    this.ros.callOnConnection(message);
    
    return {
      goalId: goalId,
      cancel: () => this.cancelGoal(goalId)
    };
  }
  
  cancelGoal(goalId) {
    const goalInfo = this.activeGoals.get(goalId);
    if (!goalInfo) return;
    
    this.ros.callOnConnection({
      op: 'cancel_action_goal',
      action: this.actionName,
      id: goalId
    });
    
    // Limpiar listeners (nota: necesitarás guardar referencias a los listeners)
    this.activeGoals.delete(goalId);
  }
  
  cancelAll() {
    for (const [goalId, _] of this.activeGoals) {
      this.cancelGoal(goalId);
    }
  }
}