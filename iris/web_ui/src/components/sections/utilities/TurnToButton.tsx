// TurnToButton.tsx
import { useState } from 'react';
import { sendPost } from '../../../hooks/useSendPost';

type Props = {
  label: string;
  value: number;
  robotId: number;
};

export default function TurnToButton({ label, value, robotId }: Props) {
  const [x, setX] = useState<number>(0);
  const [y, setY] = useState<number>(0);

  const handleClick = async () => {
    await sendPost('http://localhost:5000/data', { skill: 0, robot_id: robotId });
    await sendPost('http://localhost:5000/data', {
      skill: value,
      robot_id: robotId,
      turn_to_x: x,
      turn_to_y: y,
    });
  };

  return (
    <div className="mb-2">
      <div className="grid grid-cols-2 gap-2 mb-2">
        <div className="flex flex-col">
          <label className="mb-1 font-semibold">X</label>
          <input
            type="number"
            value={x}
            onChange={(e) => setX(parseFloat(e.target.value))}
            placeholder="X"
            className="p-1 rounded border"
          />
        </div>
        <div className="flex flex-col">
          <label className="mb-1 font-semibold">Y</label>
          <input
            type="number"
            value={y}
            onChange={(e) => setY(parseFloat(e.target.value))}
            placeholder="Y"
            className="p-1 rounded border"
          />
        </div>
      </div>
      <button
        onClick={handleClick}
        className="w-full py-2 px-4 rounded-lg font-semibold mb-2 bg-white hover:bg-[#acacac] text-black transition"
      >
        {label}
      </button>
    </div>
  );
}
