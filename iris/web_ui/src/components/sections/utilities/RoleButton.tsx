import { sendPost } from '../../../hooks/useSendPost';

type Props = {
  label: string;
  value: number;
  color: 'default';
  robotId: number;
};

export default function RoleButton({ label, value, color, robotId }: Props) {
  const handleClick = async () => {
    console.log('Enviando role', value, 'para robô', robotId);
    await sendPost('http://localhost:5000/command', {
      role: value,
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
