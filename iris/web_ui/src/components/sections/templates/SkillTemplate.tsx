import SkillButton from '../utilities/SkillButton';
import type { Props } from '../utilities/RobotTabs';
import MoveToButton from '../utilities/MoveToButton';
import TurnToButton from '../utilities/TurnToButton';

export default function SkillTemplate({ robotId }: Props) {
  return (
    <div className="grid grid-cols-1 gap-2">
      <SkillButton label="UNKNOWN" robotId={robotId} value={0} />
      <SkillButton label="CUSHION" robotId={robotId} value={1} />
      <SkillButton label="KICK" robotId={robotId} value={2} />
      <SkillButton label="STOP" robotId={robotId} value={4} />
      <MoveToButton label="MOVE TO" robotId={robotId} value={3} />
      <TurnToButton label="TURN TO" robotId={robotId} value={5} />
    </div>
  );
}
