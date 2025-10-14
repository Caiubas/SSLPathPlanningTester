import { useEffect, useState } from 'react';
import { DataView } from './components/DataView';
import { FieldView } from './components/FieldView';
import { MenuBar } from './components/MenuBar';
import { DataViewAll } from './components/DataViewAll';
import { FIELD_DIMENSIONS, type Division } from './data/fieldDimensions';
import { initialData } from './data/initialData';
import { useFetchLoop } from './hooks/useFetchLoop';
import { mapBallToFieldCoords, mapRobotsToFieldCoords } from './utils';
import type { DataType } from './types';
import { useSendLoop } from './hooks/useSendLoop';

export type SoftwareOption = 'ia' | 'gc' | 'vision' | 'tartarus' | 'caronte' | 'robot' | 'irisGC' | 'competition';

export default function App() {
  // App.tsx
  const [flipField, setFlipField] = useState<boolean>(false);
  const [receptDimensions, setReceptDimensions] = useState<boolean>(false);

  const [selectedSoftware, setSelectedSoftware] =
    useState<keyof DataType>('tartarus');
  const [reading, setReading] = useState(false);
  const [selectedDivision, setSelectedDivision] =
    useState<Division>('Entry Level');

  const data = useFetchLoop(reading, initialData);
  useSendLoop(reading, data);
  

  let dimensions = initialData.vision.field;

  if (receptDimensions) {
    dimensions = data.vision.field;
  } else {
    dimensions = FIELD_DIMENSIONS[selectedDivision];
  }

  const totalFieldLength = dimensions.field_length + 2 * dimensions.goal_depth;
  const centerX = dimensions.field_width / 2;
  const centerY = totalFieldLength / 2;

  const yellowRobots = data.vision.robots_yellow
    .filter((r) => r.detected == true)
    .map((r) => mapRobotsToFieldCoords([r], centerX, centerY)[0]);

  const blueRobots = data.vision.robots_blue
    .filter((r) => r.detected == true)
    .map((r) => mapRobotsToFieldCoords([r], centerX, centerY)[0]);

  const ball = mapBallToFieldCoords(data.vision.balls, centerX, centerY);
  const [selectedRobotId, setSelectedRobotId] = useState<number | null>(null);
  const [isFullScreen, setIsFullScreen] = useState<boolean>(window.innerWidth > 1200);

  useEffect(() => {
    const handleResize = () => {
      setIsFullScreen(window.innerWidth > 1200);
    };

    window.addEventListener('resize', handleResize);
    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, []);

  return (
    <div className="bg-[#311A52] h-screen w-screen overflow-y-scroll">
      <div className="flex flex-col h-screen w-screen">
        <div className="h-auto">
          <MenuBar
            onSelectSoftware={setSelectedSoftware}
            onSelectDivision={setSelectedDivision}
          />
        </div>

        <div className={`flex overflow-y-scroll overflow-x-hidden ${
            isFullScreen ? 'flex-row' : 'flex-col'
          }`}>
          {/* Campo ajustado pela divisão selecionada */}
          <FieldView
            data={data}
            dimensions={dimensions}
            blueRobots={blueRobots}
            yellowRobots={yellowRobots}
            ball={ball}
            flipField={flipField}
          />

           <div className={`flex gap-2 w-full ${
            isFullScreen ? 'flex-row flex-nowrap ' : 'flex-col flex-wrap '
          }`}>
            <DataView data={data} reading={reading} setReading={setReading} 
                  className=
                    {`${
                    window.innerWidth > 768 ? "flex-1" : "flex-[1]" // if para mobile (metade da largura)
                     }`} 
                      />
            {selectedSoftware && (
              <DataViewAll
                   className={`${
                  window.innerWidth < 768 ? "flex-1" : "flex-[2]" // if para desktop (2x maior que o DataView)
                }`}
                reading={reading}
                selected={selectedSoftware}
                setSelected={setSelectedSoftware}
                flipField={flipField}
                setFlipField={setFlipField}
                receptDimensions={receptDimensions}
                setReceptDimensions={setReceptDimensions}
                selectedRobotId={selectedRobotId}
                setSelectedRobotId={setSelectedRobotId}
              />
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
