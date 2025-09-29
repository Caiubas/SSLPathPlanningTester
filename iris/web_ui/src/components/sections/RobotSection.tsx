import type { DataType } from '../../types';
import RobotTabs from './utilities/RobotTabs';

type Props = {
  data: DataType;
  robotId: number;
  setSelected: React.Dispatch<React.SetStateAction<keyof DataType>>;
};

export default function RobotSection({ data, robotId, setSelected }: Props) {
  const teamBlueSelected = data.gc.team_blue;
  const team = teamBlueSelected ? 'blue_team' : 'yellow_team';

  return (
    <div>
      {/* Botão para voltar para Hades */}
      <button
        className="mb-2 px-4 py-2 bg-purple-600 rounded hover:bg-purple-700"
        onClick={() => setSelected('ia')}
      >
        Voltar para Hades
      </button>

      <img
        src={`/img/${team}/id${robotId}.png`}
        alt={`Robô ${robotId}`}
        className="w-14 h-14 object-contain shrink-0"
      />

      <h2 className="text-lg font-bold mb-1">Skills do Robô {robotId}</h2>
      {/* Aqui você renderiza as skills do robô */}
      <RobotTabs robotId={robotId}/>
    </div>
  );
}
