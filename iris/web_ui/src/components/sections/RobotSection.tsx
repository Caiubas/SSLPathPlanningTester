import { useState, useEffect } from 'react';
import type { DataType } from '../../types';
import { toggleBooleanWithId } from '../../utils';
import RobotTabs from './utilities/RobotTabs';
import { RowWrapper } from './utilities/RowWrapper';
import { ToggleSwitch } from './utilities/ToggleSwitch';

type Props = {
  data: DataType;
  robotId: number;
  setSelected: React.Dispatch<React.SetStateAction<keyof DataType>>;
};

export default function RobotSection({ data, robotId, setSelected }: Props) {
  const teamBlueSelected = data.gc.team_blue;
  const team = teamBlueSelected ? 'blue_team' : 'yellow_team';

  // Estado local para o switch
  const [hasKicker, setHasKicker] = useState(data.robot.has_kicker);

  // Sincroniza caso os dados externos mudem (ex: LCM atualizando)
  useEffect(() => {
    setHasKicker(data.robot.has_kicker);
  }, [data.robot.has_kicker]);

  const handleToggle = async () => {
    const newValue = !hasKicker;
    setHasKicker(newValue); // atualiza o front imediatamente
    await toggleBooleanWithId('has_kicker', hasKicker, robotId);
  };

  // No topo do RobotSection
  // Forçando que seja um objeto indexado por number
  // Map de skills por robô
  const skillNames: Record<number, string> = {
    0: 'Nenhuma',
    1: 'Cushion',
    2: 'Kick',
    3: 'Move To',
    4: 'Stop',
    5: 'Turn To',
  };

  // Garante que robot existe
  const robot = data.robot; // ou data.skill se você voltou para esse
  if (!robot) return null; // evita undefined

  const currentSkill = robot.skill_robot;
  const currentSkillName = skillNames[currentSkill] ?? 'Desconhecida';

  console.log('Skill atual:', currentSkill, currentSkillName, robot);

  return (
    <div>
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

      <RowWrapper title="Has Kicker">
        <ToggleSwitch value={hasKicker} onToggle={handleToggle} />
      </RowWrapper>

      <RowWrapper title="Skill Atual">
        <span>{currentSkillName}</span>
      </RowWrapper>

      <h2 className="text-lg font-bold mb-1">Skills do Robô {robotId}</h2>
      <RobotTabs robotId={robotId} />
    </div>
  );
}
