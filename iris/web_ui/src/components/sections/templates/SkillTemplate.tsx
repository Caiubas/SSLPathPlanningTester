import SkillButton from '../utilities/SkillButton';
import type { Props } from '../utilities/RobotTabs';

export default function SkillTemplate({ robotId }: Props) {
  return (
    <>
      <div className="grid grid-cols-3 gap-1">
        <SkillButton label="TOUCH LINE" robotId={robotId} value={6} color="default" />
      </div>
    </>
  );
}
