import type { DataType, DetectionRobot } from '../../types';

type Props = {
  data: DataType;
};

function convertRobots(robots: DetectionRobot[] | undefined): DetectionRobot[] {
  if (!robots) return [];
  return robots.map((dr) => ({
    robot_id: dr.robot_id,
    // mm -> m
    position_x: dr.position_x,
    position_y: dr.position_y,
    // mantém em radianos
    orientation: dr.orientation,
    detected: dr.detected,
  }));
}

export default function VisionSection({ data }: Props) {
  const vision = data?.vision;

  /*const yellowRobots: DetectionRobot[] = [
    {
      robot_id: 0,
      position_x: 1.25,
      position_y: 0.75,
      orientation: 1.57,
      detected: true,
    },
    {
      robot_id: 1,
      position_x: -0.4,
      position_y: 1.2,
      orientation: -0.25,
      detected: true,
    },
    {
      robot_id: 2,
      position_x: 0.0,
      position_y: 0.0,
      orientation: 0.0,
      detected: false,
    },
  ];

  // Mock de robôs azuis
  const blueRobots: DetectionRobot[] = [
    {
      robot_id: 3,
      position_x: -1.1,
      position_y: -0.8,
      orientation: 3.14,
      detected: true,
    },
    {
      robot_id: 4,
      position_x: 0.9,
      position_y: -1.5,
      orientation: 2.45,
      detected: true,
    },
    {
      robot_id: 5,
      position_x: 0.2,
      position_y: -0.3,
      orientation: -1.0,
      detected: false,
    },
  ]; */
  const yellowRobots: DetectionRobot[] = convertRobots(vision?.robots_yellow);
  const blueRobots: DetectionRobot[] = convertRobots(vision?.robots_blue);

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
              {vision.balls.position_x.toFixed(2)} mm
            </span>
          </p>
          <p>
            Position Y:{' '}
            <span className="font-mono">
              {vision.balls.position_y.toFixed(2)} mm
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
            .sort((a, b) => a.robot_id - b.robot_id)
            .map((robot) => (
              <div key={robot.robot_id} className="mb-2">
                <p>
                  Id: <span className="font-mono">{robot.robot_id}</span>
                </p>
                <p>
                  Pos X:{' '}
                  <span className="font-mono">{robot.position_x.toFixed(2)} mm</span>
                </p>
                <p>
                  Pos Y:{' '}
                  <span className="font-mono">{robot.position_y.toFixed(2)} mm</span>
                </p>
                <p>
                  Orientation:{' '}
                  <span className="font-mono">
                    {robot.orientation.toFixed(3)} rad
                  </span>
                </p>
              </div>
            ))
        ) : (
          <p className="italic text-gray-500">
            Nenhum robô amarelo identificado.
          </p>
        )}
      </div>

      {/* Robôs azuis */}
      <h3 className="text-md font-semibold mt-2">Robôs Azuis</h3>
      <div className="max-h-[250px] overflow-y-auto border border-[#6805F2] rounded p-2 bg-[#2E2E2E]">
        {blueRobots.length > 0 ? (
          blueRobots
            .sort((a, b) => a.robot_id - b.robot_id)
            .map((robot) => (
              <div key={robot.robot_id} className="mb-2">
                <p>
                  ID: <span className="font-mono">{robot.robot_id}</span>
                </p>
                <p>
                  Pos X:{' '}
                  <span className="font-mono">{robot.position_x.toFixed(2)} m</span>
                </p>
                <p>
                  Pos Y:{' '}
                  <span className="font-mono">{robot.position_y.toFixed(2)} m</span>
                </p>
                <p>
                  Orientation:{' '}
                  <span className="font-mono">
                    {robot.orientation.toFixed(3)} rad
                  </span>
                </p>
              </div>
            ))
        ) : (
          <p className="italic text-gray-500">Nenhum robô azul identificado.</p>
        )}
      </div>
    </>
  );
}
