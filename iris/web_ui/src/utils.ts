import { competitionData } from './data/competitionData';
import { sendPost } from './hooks/useSendPost';
import type { DataType, DetectionBall, DetectionRobot } from './types';

// -------------------- Conversões --------------------

export function detectionRobot(
  robots: DetectionRobot[] | undefined,
): DetectionRobot[] {
  if (!robots) return [];
  return robots.map((dr) => ({
    robot_id: dr.robot_id,
    position_x: dr.position_x,
    position_y: dr.position_y,
    orientation: dr.orientation,
    detected: dr.detected,
  }));
}

export function mapRobotsToFieldCoords(
  robots: DetectionRobot[] | undefined,
  centerX: number,
  centerY: number,
): DetectionRobot[] {
  if (!robots) return [];
  return robots.map((dr) => ({
    robot_id: dr.robot_id,
    position_x: centerX + dr.position_y,
    position_y: centerY + dr.position_x,
    orientation: ((dr.orientation * 180) / Math.PI - 90 + 360) % 360,
    detected: dr.detected,
  }));
}

export function mapBallToFieldCoords(
  ball: DetectionBall | undefined,
  centerX: number,
  centerY: number
): { x: number; y: number } | undefined {
  if (!ball) return undefined;
  return {
    x: centerX + ball.position_y,
    y: centerY + ball.position_x,
  };
}

// -------------------- Filtros --------------------


// -------------------- Toggles --------------------

export const toggleLocal = async (
  key: keyof DataType['tartarus'],
  value: boolean,
  setValue: React.Dispatch<React.SetStateAction<boolean>>
) => {
  try {
    const newValue = !value;
    setValue(newValue);

    const success = await sendPost('http://localhost:5000/data', {
      [key]: newValue,
    });

    if (!success) {
      console.error(`Erro ao alternar ${key}`);
      setValue(value); // rollback se deu erro
    }
  } catch (err) {
    console.error(`Erro ao enviar ${key}:`, err);
    setValue(value); // rollback
  }
};

let half_field = false; // variável global/local

export const toggleBoolean = async (key: string, currentValue: boolean) => {
  try {
    let payload;
    let newValue = !currentValue;

    if (key === 'competition_mode') {
      if (!currentValue) {
        payload = competitionData;
      } else {
        payload = { competition_mode: false };
      }
    }

    else if (key === 'half_field') {
      half_field = !half_field;
      newValue = half_field;
      payload = { [key]: newValue };
      console.log("half_field agora é:", half_field);
    }

    else {
      payload = { [key]: newValue };
    }

    const success = await sendPost('http://localhost:5000/data', payload);

    if (!success) {
      console.error(`Erro ao alternar ${key}`);
    }
  } catch (err) {
    console.error(`Erro ao enviar ${key}:`, err);
  }
};


export const toggleBooleanWithId = async (
  key: string,
  currentValue: boolean,
  robotId: number
) => {
  try {
    const payload = {
      robot_id: robotId,
      [key]: !currentValue,
    };

    const success = await sendPost('http://localhost:5000/data', payload);

    if (!success) {
      console.error(`Erro ao alternar ${key} para robô ${robotId}`);
    }
  } catch (err) {
    console.error(`Erro ao enviar ${key} para robô ${robotId}:`, err);
  }
};

// -------------------- Update Numérico --------------------

export const updateNumber = async (key: string, value: number) => {
  const success = await sendPost('http://localhost:5000/data', {
    [key]: value,
  });

  if (!success) {
    console.error(`Erro ao atualizar ${key} para ${value}`);
  }
};

export function getMidField() {
  return half_field;
}

// technicalMoveState.ts

export let technicalMove = 0; // valor inicial

export const setTechnicalMove = (value: number) => {
  technicalMove = value;
  console.log("Move Action selecionada:", value);
};

export const generateTrajectory = (
  from: { x: number; y: number },
  to: { x: number; y: number },
  steps: number = 20
) => {
  const points = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    points.push({
      x: from.x + t * (to.x - from.x),
      y: from.y + t * (to.y - from.y),
    });
  }
  return points;
};



