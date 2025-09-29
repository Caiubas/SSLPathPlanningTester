import { sendPost } from '../../../hooks/useSendPost';

type Props = {
  label: string;
  value: number;
  color: 'default';
  robotId: number;
};

export default function SkillButton({ label, value, color, robotId }: Props) {
  const handleClick = async () => {
    // sempre reseta para 0 antes de enviar o novo evento
    await sendPost('http://localhost:5000/command', {
      skill: 0,
      robot_id: robotId,
    });
    await sendPost('http://localhost:5000/command', {
      skill: value,
      robot_id: robotId,
    });
  };

  const bgColor =
    color === 'default'
      ? 'bg-white hover:bg-[#acacac] text-black'
      : 'bg-white hover:bg-[#acacac] text-black';

  return (
    <button
      onClick={handleClick}
      className={`w-full py-2 px-4 rounded-lg font-semibold mb-2 transition ${bgColor}`}
    >
      {label}
    </button>
  );
}
