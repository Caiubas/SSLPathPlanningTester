import type { FIELD_DIMENSIONS } from '../data/fieldDimensions';
import type { BallField, DataType, RobotField } from '../types';
import { getMidField } from '../utils';
import { FieldSVG } from './FieldSVG';
import { MidFieldSVG } from './MidFieldSVG';

type FieldDimensions = (typeof FIELD_DIMENSIONS)[keyof typeof FIELD_DIMENSIONS];

export type FieldProps = {
  data: DataType;
  dimensions: FieldDimensions;
  blueRobots?: RobotField[];
  yellowRobots?: RobotField[];
  ball?: BallField;
  flipField: boolean;
};

export function FieldView({
  data,
  dimensions,
  blueRobots = [],
  yellowRobots = [],
  ball,
  flipField,
}: FieldProps) {
  const midField = getMidField();
  return (
    <div
      className="h-full flex justify-center items-center bg-[#3B3B3B]"
      style={{
        minWidth: '500px',
        maxWidth: '1000px',
        flexShrink: 0,
        padding: '16px',
      }}
    >
      <div
        className="w-full h-full flex justify-center items-center"
        style={{
          aspectRatio:
            dimensions.field_width /
            (dimensions.field_length + 2 * dimensions.goal_depth),
        }}
      >
        {midField ? (
          <MidFieldSVG
            data={data}
            dimensions={dimensions}
            blueRobots={blueRobots}
            yellowRobots={yellowRobots}
            ball={ball}
            flipField={flipField}
          />
        ) : (
          <FieldSVG
            data={data}
            dimensions={dimensions}
            blueRobots={blueRobots}
            yellowRobots={yellowRobots}
            ball={ball}
            flipField={flipField}
          />
        )}
      </div>
    </div>
  );
}
