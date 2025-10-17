import { useState } from 'react';
import RoleTemplate from '../templates/RoleTemplate';
import SkillTemplate from '../templates/SkillTemplate';

export type RobotTabsProps = {
  robotId: number;
  currentPos?: { x: number; y: number };
  addTrajectory: (robotId: number, from: { x: number; y: number }, to: { x: number; y: number }) => void;
}

export default function RobotTabs({ robotId, currentPos, addTrajectory } : RobotTabsProps) {
  const [activeTab, setActiveTab] = useState('ball_left');

  const tabs = [
    { id: 'role', label: 'ROLES' },
    { id: 'skill', label: 'SKILLS' },
  ];

  return (
    <div className="w-full max-w-4xl mx-auto">
      {/* Cabeçalho das abas */}
      <div className="flex">
        {tabs.map((tab) => (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`flex-1 px-4 py-4 text-sm font-medium border-b-2 bg-[#6d6d6d] ${
              activeTab === tab.id
                ? 'border-[#8D00F2] text-white bg-[#494949]'
                : 'border-transparent text-[#313131] hover:text-[#000000]'
            }`}
          >
            {tab.label}
          </button>
        ))}
      </div>

      {/* Conteúdo das abas */}
      <div className="p-4 bg-[#7e7e7e]">
        {activeTab === 'role' && <RoleTemplate robotId={robotId}/>}
        {activeTab === 'skill' && <SkillTemplate robotId={robotId} currentPos={currentPos} addTrajectory={addTrajectory}/>}
      </div>
    </div>
  );
}
