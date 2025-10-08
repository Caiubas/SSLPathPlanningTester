import type { FieldProps } from './FieldView';

export type Robot = {
  id: number;
  x: number;
  y: number;
  orientation?: number; // graus, opcional
};

export function MidFieldSVG({
  data,
  dimensions,
  blueRobots = [],
  yellowRobots = [],
  ball,
  flipField = false,
}: FieldProps) {
  const allField = dimensions.field_length + 2 * dimensions.goal_depth;
  const totalFieldLength = dimensions.field_length / 2 + dimensions.goal_depth;
  const centerX = totalFieldLength;
  const centerY = dimensions.field_width / 2;
  const goalTopY = 0;
  const goalBottomY = allField - dimensions.goal_depth;
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
      viewBox={`${-dimensions.boundary_width} 0 ${dimensions.field_width} ${dimensions.field_width}`}
      preserveAspectRatio="xMidYMid meet"
    >
      <g
        transform={
          flipField ? `rotate(180, ${centerX}, ${centerY})` : undefined
        }
      >
        {/* Fundo campo + gol (centralizado com largura do gol) */}
        <rect
          x={0}
          y={(dimensions.field_width - dimensions.goal_width) / 2}
          width={allField}
          height={dimensions.goal_width}
          fill="#545454"
        />

        {/* Campo jogável */}
        
        {/* parte da esquerda */}
        <rect
          x={dimensions.goal_depth}
          y={0}
          width={dimensions.field_length / 2}
          height={dimensions.field_width}
          stroke="#0000f9"
          strokeWidth={dimensions.line_thickness}
          fill="#545454"
        />

        {/* parte da direita */}
        <rect
          x={dimensions.goal_depth + dimensions.field_length / 2}
          y={0}
          width={dimensions.field_length / 2}
          height={dimensions.field_width}
          stroke="#fefe00"
          strokeWidth={dimensions.line_thickness}
          fill="#545454"
        />

        {/* Linha do meio campo */}
        <line
          y1={0}
          x1={centerX}
          y2={dimensions.field_width}
          x2={centerX}
          stroke="white"
          strokeWidth={dimensions.line_thickness*1.5}
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
          x={dimensions.goal_depth}
          y={(dimensions.field_width - dimensions.defense_area_width) / 2}
          width={dimensions.defense_area_height}
          height={dimensions.defense_area_width}
          stroke="#0000f9"
          strokeWidth={dimensions.line_thickness}
          fill="transparent"
        />

        {/* Área de defesa inferior */}
        <rect
          y={(dimensions.field_width - dimensions.defense_area_width) / 2}
          x={
            dimensions.goal_depth +
            dimensions.field_length -
            dimensions.defense_area_height
          }
          width={dimensions.defense_area_height}
          height={dimensions.defense_area_width}
          stroke="#fefe00"
          strokeWidth={dimensions.line_thickness}
          fill="transparent"
        />

        {/* Gol superior */}
        <line
          y1={(dimensions.field_width - dimensions.goal_width) / 2}
          x1={goalTopY}
          y2={(dimensions.field_width + dimensions.goal_width) / 2}
          x2={goalTopY}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />
        <line
          y1={(dimensions.field_width - dimensions.goal_width) / 2}
          x1={goalTopY}
          y2={(dimensions.field_width - dimensions.goal_width) / 2}
          x2={goalTopY + dimensions.goal_depth}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />
        <line
          y1={(dimensions.field_width + dimensions.goal_width) / 2}
          x1={goalTopY}
          y2={(dimensions.field_width + dimensions.goal_width) / 2}
          x2={goalTopY + dimensions.goal_depth}
          stroke="#0000f9"
          strokeWidth={strokeWidth}
        />

        {/* Gol inferior */}
        <line
          y1={(dimensions.field_width - dimensions.goal_width) / 2}
          x1={goalBottomY + dimensions.goal_depth}
          y2={(dimensions.field_width + dimensions.goal_width) / 2}
          x2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />
        <line
          y1={(dimensions.field_width - dimensions.goal_width) / 2}
          x1={goalBottomY}
          y2={(dimensions.field_width - dimensions.goal_width) / 2}
          x2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />
        <line
          y1={(dimensions.field_width + dimensions.goal_width) / 2}
          x1={goalBottomY}
          y2={(dimensions.field_width + dimensions.goal_width) / 2}
          x2={goalBottomY + dimensions.goal_depth}
          stroke="#fefe00"
          strokeWidth={strokeWidth}
        />

        {/* Robôs azuis */}
        {blueRobots.map((robot) => {
          const adjustedOrientation = 180 + (robot.orientation ?? 0); // Ajusta conforme seu padrão
          return (
            <image
              key={`blue-${robot.id}`}
              href={`/img/blue_team/id${robot.id}.png`}
              x={robot.y - robotSize / 2}
              y={robot.x - robotSize / 2}
              width={robotSize}
              height={robotSize}
              transform={`rotate(${adjustedOrientation}, ${robot.y}, ${robot.x})`}
              pointerEvents="none"
            />
          );
        })}

        {yellowRobots.map((robot) => {
          // Inverter a orientação para espelhar no eixo vertical
          const adjustedOrientation = 180 + (robot.orientation ?? 0); // subtrai 90 para alinhar e inverte o ângulo

          return (
            <image
              key={`yellow-${robot.id}`}
              href={`/img/yellow_team/id${robot.id}.png`}
              x={robot.y - robotSize / 2}
              y={robot.x - robotSize / 2}
              width={robotSize}
              height={robotSize}
              transform={`rotate(${adjustedOrientation}, ${robot.y}, ${robot.x})`}
              pointerEvents="none"
            />
          );
        })}

        {/* Bola */}
        {ball && (
          <image
            href="/img/ball.png"
            y={dimensions.field_width - ball.x - ballSize / 2}
            x={ball.y - ballSize / 2}
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
      </g>
    </svg>
  );
}
