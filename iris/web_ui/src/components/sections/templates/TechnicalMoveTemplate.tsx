import TechnicalMoveButton from '../utilities/buttons/TechnicalMoveButton';
import type { Props } from '../utilities/RobotTabs';

export default function TechnicalMoveTemplate({ robotId }: Props) {
  return (
    <>
      <p>Basic Roles:</p>
      <div className="mt-2 mb-3 grid grid-cols-3 gap-1">
        <TechnicalMoveButton label="DIAGONAL" value={1} color="default" robotId={robotId} />
      </div>
    </>
  );
}
