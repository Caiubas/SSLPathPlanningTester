import SkillButton from '../utilities/buttons/SkillButton';
import type { RobotTabsProps } from '../utilities/RobotTabs';
import MoveToButton from '../utilities/buttons/MoveToButton';
import TurnToButton from '../utilities/buttons/TurnToButton';

export default function SkillTemplate({ robotId, currentPos, addTrajectory }: RobotTabsProps) {
  return (
    <div className="grid grid-cols-1 gap-2">
      <SkillButton label="UNKNOWN" robotId={robotId} value={0} />
      <SkillButton label="CUSHION" robotId={robotId} value={1} />
      <SkillButton label="KICK" robotId={robotId} value={2} />
      <SkillButton label="STOP" robotId={robotId} value={4} />
      <MoveToButton label="MOVE TO" robotId={robotId} value={3} currentPos={currentPos} addTrajectory={addTrajectory} />
      <TurnToButton label="TURN TO" robotId={robotId} value={5} />
    </div>
  );
}
