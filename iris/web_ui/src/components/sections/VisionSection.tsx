import type { DataType, DetectionRobot, RobotField } from '../../types';

type Props = {
  data: DataType;
};

function convertRobots(robots: DetectionRobot[] | undefined): RobotField[] {
  if (!robots) return [];
  return robots.map((dr) => ({
    id: dr.robot_id,
    // mm -> m
    x: dr.position_x,
    y: dr.position_y,
    // mantém em radianos
    orientation: dr.orientation,
  }));
}

export default function VisionSection({ data }: Props) {
  const vision = data?.vision;

  const yellowRobots: RobotField[] = convertRobots(vision?.robots_yellow);
  const blueRobots: RobotField[] = convertRobots(vision?.robots_blue);

  return (
    <>
      <h2 className="text-lg font-bold mb-1">Vision</h2>

      {/* Bola */}
      <h3 className="text-md font-semibold mt-2">Bola</h3>
      {vision?.balls ? (
        <>
          <p>
            Position X:{' '}
            <span className="font-mono">
              {(vision.balls.position_x).toFixed(2)} mm
            </span>
          </p>
          <p>
            Position Y:{' '}
            <span className="font-mono">
              {(vision.balls.position_y).toFixed(2)} mm
            </span>
          </p>
        </>
      ) : (
        <p className="italic text-gray-500">Dados da bola indisponíveis.</p>
      )}

      {/* Campo */}
      <h3 className="text-md font-semibold mt-2">Campo</h3>
      {vision?.field ? (
        Object.entries(vision.field).map(([key, value]) => (
          <p key={key}>
            {key.replace(/_/g, ' ').replace(/^\w/, (c) => c.toUpperCase())}:{' '}
            <span className="font-mono">{value}</span>
          </p>
        ))
      ) : (
        <p className="italic text-gray-500">Dados de campo indisponíveis.</p>
      )}

      {/* Robôs amarelos */}
      <h3 className="text-md font-semibold mt-2">Robôs Amarelos</h3>
      <div className="max-h-[250px] overflow-y-auto border border-[#6805F2] rounded p-2 bg-[#2E2E2E]">
        {yellowRobots.length > 0 ? (
          yellowRobots
            .sort((a, b) => a.id - b.id)
            .map((robot) => (
              <div key={robot.id} className="mb-2">
                <p>ID: <span className="font-mono">{robot.id}</span></p>
                <p>Pos X: <span className="font-mono">{robot.x.toFixed(2)} mm</span></p>
                <p>Pos Y: <span className="font-mono">{robot.y.toFixed(2)} mm</span></p>
                <p>Orientation: <span className="font-mono">{robot.orientation.toFixed(3)} rad</span></p>
              </div>
            ))
        ) : (
          <p className="italic text-gray-500">Nenhum robô amarelo identificado.</p>
        )}
      </div>

      {/* Robôs azuis */}
      <h3 className="text-md font-semibold mt-2">Robôs Azuis</h3>
      <div className="max-h-[250px] overflow-y-auto border border-[#6805F2] rounded p-2 bg-[#2E2E2E]">
        {blueRobots.length > 0 ? (
          blueRobots
            .sort((a, b) => a.id - b.id)
            .map((robot) => (
              <div key={robot.id} className="mb-2">
                <p>ID: <span className="font-mono">{robot.id}</span></p>
                <p>Pos X: <span className="font-mono">{robot.x.toFixed(2)} m</span></p>
                <p>Pos Y: <span className="font-mono">{robot.y.toFixed(2)} m</span></p>
                <p>Orientation: <span className="font-mono">{robot.orientation.toFixed(3)} rad</span></p>
              </div>
            ))
        ) : (
          <p className="italic text-gray-500">Nenhum robô azul identificado.</p>
        )}
      </div>
    </>
  );
}
