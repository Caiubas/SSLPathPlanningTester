import SkillButton from '../utilities/SkillButton';
import type { Props } from '../utilities/RobotTabs';
<<<<<<< HEAD

export default function SkillTemplate({ robotId }: Props) {
  return (
    <>
      <div className="grid grid-cols-3 gap-1">
        <SkillButton label="TOUCH LINE" robotId={robotId} value={6} color="default" />
      </div>
    </>
=======
import MoveToButton from '../utilities/MoveToButton';
import TurnToButton from '../utilities/TurnToButton';

export default function SkillTemplate({ robotId }: Props) {
  return (
    <div className="grid grid-cols-1 gap-2">
      <SkillButton label="Cushion" robotId={robotId} value={1} />
      <SkillButton label="Kick" robotId={robotId} value={2} />
      <SkillButton label="Stop" robotId={robotId} value={4} />
      <MoveToButton label="Move To" robotId={robotId} value={3} />
      <TurnToButton label="Turn To" robotId={robotId} value={5} />
    </div>
>>>>>>> d8c60bd04d9e6a0a6853043723cb8829b6576bc8
  );
}
