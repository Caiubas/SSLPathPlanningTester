import RoleButton from '../utilities/buttons/RoleButton';
import type { Props } from '../utilities/RobotTabs';

export default function RoleTemplate({ robotId }: Props) {
  return (
    <>
      <p>Basic Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-3 gap-1">
        <RoleButton label="UNKNOWN" value={-1} color="default" robotId={robotId} />
        <RoleButton label="GOAL_KEEPER" value={0} color="default" robotId={robotId} />
        <RoleButton label="STRIKER" value={1} color="default" robotId={robotId} />
        <RoleButton label="SUPPORT" value={2} color="default" robotId={robotId} />
        <RoleButton label="DEFENDER" value={3} color="default" robotId={robotId} />
        <RoleButton label="HALTED" value={4} color="default" robotId={robotId} />
      </div>

      <p>Kickoff Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-3 gap-1">
        <RoleButton label="KICKOFF_KICKER" value={5} color="default" robotId={robotId} />
        <RoleButton label="KICKOFF_GOAL_KEEPER" value={6} color="default" robotId={robotId} />
        <RoleButton label="KICKOFF_SUPPORT" value={7} color="default" robotId={robotId} />
      </div>

      <p>Special Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-3 gap-1">
        <RoleButton label="MARKER" value={8} color="default" robotId={robotId} />
        <RoleButton label="RETAKER" value={11} color="default" robotId={robotId} />
        <RoleButton label="PENALTIER" value={12} color="default" robotId={robotId} />
        <RoleButton label="FREEKICKER" value={13} color="default" robotId={robotId} />
        <RoleButton label="WATCHER" value={14} color="default" robotId={robotId} />
      </div>

      <p>Debug Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-2 gap-1">
        <RoleButton label="DEBUG_CIRCULAR_TRAJECTORY" value={9} color="default" robotId={robotId} />
        <RoleButton label="DEBUG_SQUARED_TRAJECTORY" value={10} color="default" robotId={robotId} />
      </div>

      <p>Placeholder Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-2 gap-1">
        <RoleButton label="PLACEHOLDER" value={15} color="default" robotId={robotId} />
        <RoleButton label="PLACER" value={16} color="default" robotId={robotId} />
      </div>
    </>
  );
}
