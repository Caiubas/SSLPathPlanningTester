import type { FieldProps } from './FieldView';

export function FieldSVG({
  data,
  dimensions,
  blueRobots = [],
  yellowRobots = [],
  ball,
  flipField = false,
  trajectories,
}: FieldProps) {
  const totalFieldLength = dimensions.field_length + 2 * dimensions.goal_depth;
  const centerX = dimensions.field_width / 2;
  const centerY = totalFieldLength / 2;
  const goalTopY = 0;
  const goalBottomY = totalFieldLength - dimensions.goal_depth;
  const strokeWidth = 10;
  const robotSize = dimensions.max_robot_radius * 2;
  const ballSize = dimensions.ball_radius * 2;

  const designatedPosition = data.tartarus.iris_as_GC
    ? {
      x: data?.irisGC?.designated_position_x ?? 0,
      y: data?.irisGC?.designated_position_y ?? 0,
    }
    : {
      x: data?.gc?.gc_designated_position_x ?? 0,
      y: data?.gc?.gc_designated_position_y ?? 0,
    };

  console.log(
    'flipField:',
    flipField,
    'centerX:',
    centerX,
    'centerY:',
    centerY,
  );

  return (
    <svg
      className="h-full w-auto"
      viewBox={`${-dimensions.boundary_width} 0 ${dimensions.field_width + 2 * dimensions.boundary_width} ${totalFieldLength}`}
      preserveAspectRatio="xMidYMid meet"
    >
      <g
        transform={
          flipField ? `rotate(180, ${centerX}, ${centerY})` : undefined
        }
      >
        {/* Fundo campo + gol (centralizado com largura do gol) */}
        <rect
          x={(dimensions.field_width - dimensions.goal_width) / 2}
          y={0}
          width={dimensions.goal_width}
          height={totalFieldLength}
          fill="#545454"
        />

        {/* Campo jogável */}
        {/* parte de cima */}
        <rect
          x={0}
          y={dimensions.goal_depth}
          width={dimensions.field_width}
          height={dimensions.field_length / 2}
          stroke="#0000f9"
          strokeWidth={dimensions.line_thickness}
          fill="#545454"
        />

        {/* parte de baixo */}
        <rect
          x={0}
          y={dimensions.goal_depth + dimensions.field_length / 2}
          width={dimensions.field_width}
          height={dimensions.field_length / 2}
          stroke="#fefe00"
          strokeWidth={dimensions.line_thickness}
          fill="#545454"
        />

        {/* Linha do meio campo */}
        <line
          x1={0}
          y1={centerY}
          x2={dimensions.field_width}
          y2={centerY}
          stroke="white"
          strokeWidth={dimensions.line_thickness * 1.5}
        />

        {/* Circulo central */}
        <circle
          cx={centerX}
          cy={centerY}
          r={dimensions.center_circle_radius}
          stroke="white"
          strokeWidth={dimensions.line_thickness}
          fill="transparent"
        />

        {/* Área de defesa superior */}
        <rect
          x={(dimensions.field_width - dimensions.defense_area_width) / 2}
          y={dimensions.goal_depth}
          width={dimensions.defense_area_width}
          height={dimensions.defense_area_height}
          stroke="#0000f9"
          strokeWidth={dimensions.line_thickness}
          fill="transparent"
        />

        {/* Área de defesa inferior */}
        <rect
          x={(dimensions.field_width - dimensions.defense_area_width) / 2}
          y={
            dimensions.goal_depth +
            dimensions.field_length -
            dimensions.defense_area_height
          }
          width={dimensions.defense_area_width}
          height={dimensions.defense_area_height}
          stroke="#fefe00"
          strokeWidth={dimensions.line_thickness}
          fill="transparent"
        />

        {/* Gol superior */}
        <line
          x1={(dimensions.field_width - dimensions.goal_width) / 2}
          y1={goalTopY}
          x2={(dimensions.field_width + dimensions.goal_width) / 2}
          y2={goalTopY}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />
        <line
          x1={(dimensions.field_width - dimensions.goal_width) / 2}
          y1={goalTopY}
          x2={(dimensions.field_width - dimensions.goal_width) / 2}
          y2={goalTopY + dimensions.goal_depth}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />
        <line
          x1={(dimensions.field_width + dimensions.goal_width) / 2}
          y1={goalTopY}
          x2={(dimensions.field_width + dimensions.goal_width) / 2}
          y2={goalTopY + dimensions.goal_depth}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />

        {/* Gol inferior */}
        <line
          x1={(dimensions.field_width - dimensions.goal_width) / 2}
          y1={goalBottomY + dimensions.goal_depth}
          x2={(dimensions.field_width + dimensions.goal_width) / 2}
          y2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />
        <line
          x1={(dimensions.field_width - dimensions.goal_width) / 2}
          y1={goalBottomY}
          x2={(dimensions.field_width - dimensions.goal_width) / 2}
          y2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />
        <line
          x1={(dimensions.field_width + dimensions.goal_width) / 2}
          y1={goalBottomY}
          x2={(dimensions.field_width + dimensions.goal_width) / 2}
          y2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />

        {/* Robôs azuis */}
        {blueRobots.map((robot) => {
          
          const adjustedOrientation = 90 - (robot.orientation ?? 0); // Ajusta conforme seu padrão
          return (
            <image
              key={`blue-${robot.robot_id}`}
              href={`/img/blue_team/id${robot.robot_id}.png`}
              x={robot.position_x - robotSize / 2}
              y={robot.position_y - robotSize / 2}
              width={robotSize}
              height={robotSize}
              transform={`rotate(${adjustedOrientation}, ${robot.position_x}, ${robot.position_y})`}
              pointerEvents="none"
            />
          );
        })}

        {yellowRobots.map((robot) => {
          // Inverter a orientação para espelhar no eixo vertical
          const adjustedOrientation = 90 - (robot.orientation ?? 0); // subtrai 90 para alinhar e inverte o ângulo

          return (
            <image
              key={`yellow-${robot.robot_id}`}
              href={`/img/yellow_team/id${robot.robot_id}.png`}
              x={robot.position_x - robotSize / 2}
              y={robot.position_y - robotSize / 2}
              width={robotSize}
              height={robotSize}
              transform={`rotate(${adjustedOrientation}, ${robot.position_x}, ${robot.position_y})`}
              pointerEvents="none"
            />
          );
        })}

        {/* Bola */}
        {ball && (
          <image
            href="/img/ball.png"
            x={ball.x - ballSize / 2}
            y={ball.y - ballSize / 2}
            width={ballSize}
            height={ballSize}
            pointerEvents="none"
          />
        )}

        {/* X de posição designada */}
        {designatedPosition &&
          designatedPosition.x != 0 &&
          designatedPosition.y != 0 && (
            <>
              <line
                x1={designatedPosition.x - 50}
                y1={designatedPosition.y - 50}
                x2={designatedPosition.x + 50}
                y2={designatedPosition.y + 50}
                stroke="red"
                strokeWidth={7}
              />
              <line
                x1={designatedPosition.x - 50}
                y1={designatedPosition.y + 50}
                x2={designatedPosition.x + 50}
                y2={designatedPosition.y - 50}
                stroke="red"
                strokeWidth={7}
              />
            </>
          )}

        {/*Trajetórias*/}
        {/*{trajectories?.map((traj) => (
          <g key={traj.robotId}>
            {// Linha da trajetória }
            <polyline
              points={traj.points.map((p) => `${p.x},${p.y}`).join(' ')}
              stroke={traj.robotId % 2 === 0 ? 'blue' : 'yellow'} // cor por robô/time
              strokeWidth={5}
              fill="none"
              opacity={0.6}
            />

            {// Pontos da trajetória }
            {traj.points.map((p, i) => (
              <circle
                key={i}
                cx={p.x}
                cy={p.y}
                r={6} // raio em pixels
                fill="red"
                stroke="white"
                strokeWidth={1.5}
              />
            ))}
          </g>
          ))}*/}
      </g>
    </svg>
  );
}
