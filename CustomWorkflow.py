from aeroagentsim.core.workflow import Workflow
from aeroagentsim.core.trigger import StateTrigger, TimeTrigger

class CustomWorkflow(Workflow):
    """Custom workflow implementation."""

    def __init__(self, env, name, owner, **kwargs):
        super().__init__(env, name, owner, **kwargs)

        # Register workflow properties
        self.register_property_template('target_location',
                                      value_type=list,
                                      required=True)
        self.register_property_template('timeout_duration',
                                      value_type=float,
                                      required=False)

    def _setup_transitions(self):
        """Define state machine transitions."""
        sm = self.status_machine

        # Set initial state after workflow starts
        sm.set_start_transition('moving')

        # Transition: Arrive at target location
        sm.add_transition(
            state='moving',
            next_status='working',
            agent_state={
                'agent_id': self.owner.id,
                'state_key': 'position',
                'operator': TriggerOperator.CUSTOM,
                'target_value': self._at_target_location
            },
            description="Arrived at target location"
        )

        # Transition: Complete work after time delay
        sm.add_transition(
            state='working',
            next_status='completed',
            time_trigger={'delay': 300},  # 5 minutes
            description="Work completed"
        )

    def _at_target_location(self, position):
        """Check if agent is at target location."""
        target = self.properties['target_location']
        distance = self._calculate_distance(position, target)
        return distance < 5.0  # Within 5 meters
