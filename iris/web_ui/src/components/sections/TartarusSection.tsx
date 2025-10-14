import type { DataType } from '../../types';
import { RowWrapper } from './utilities/RowWrapper';
import { ToggleSwitch } from './utilities/ToggleSwitch';
import { NumberInputRow } from './utilities/NumberInputRow';
import { ActionButton } from './utilities/ActionButton';
import { CompetitionOverlay } from './utilities/CompetitionOverlay';
import GoalkeeperIdInput from './utilities/GoalkeeperIdInput';
import { useTartarusState } from '../../hooks/useTartarusState';
import { getMidField, toggleBoolean, updateNumber } from '../../utils';
import { sendPost } from '../../hooks/useSendPost';

type Props = {
  data: DataType;
  flipField: boolean;
  receptDimensions: boolean;
  setFlipField: React.Dispatch<React.SetStateAction<boolean>>;
  setReceptDimensions: React.Dispatch<React.SetStateAction<boolean>>;
};

export default function TartarusSection({
  data,
  setFlipField,
  flipField,
  setReceptDimensions,
  receptDimensions,
}: Props) {
  const { tartarus } = data;

  const value = getMidField();

  const {
    stmPort,
    setStmPort,
    mcastGCPort,
    setMcastPortGC,
    mcastSslVisionPort,
    setSslVisionPort,
    mcastTrackedPort,
    setTrackedPort,
    camsNumber,
    setCamsNumber,
    mcastGrsimPort,
    setMcastGrsimPort,
  } = useTartarusState(tartarus);

  return (
    <div className="relative">
      {/* Overlay só aparece se competition_mode estiver ativo */}
      {tartarus.competition_mode && (
        <CompetitionOverlay imageSrc={`/img/Venom.png`} />
      )}

      <h2 className="text-lg font-bold mb-4">Tartarus</h2>

      {/* Toggles */}
      <RowWrapper title="SSL Vision:">
        <ToggleSwitch
          value={tartarus.ssl_vision}
          onToggle={() => toggleBoolean('ssl_vision', tartarus.ssl_vision)}
        />
      </RowWrapper>

      <RowWrapper title="Auto Referee:">
        <ToggleSwitch
          value={tartarus.autoreferee}
          onToggle={() => toggleBoolean('autoreferee', tartarus.autoreferee)}
        />
      </RowWrapper>

      <RowWrapper title="Modo Competição:">
        <ToggleSwitch
          value={tartarus.competition_mode}
          onToggle={() =>
            toggleBoolean('competition_mode', tartarus.competition_mode)
          }
        />
      </RowWrapper>

      <RowWrapper title="Iris GC:">
        <ToggleSwitch
          value={tartarus.iris_as_GC}
          onToggle={() => toggleBoolean('iris_as_GC', tartarus.iris_as_GC)}
        />
      </RowWrapper>

      <RowWrapper title="Controlar o robô em:">
        <select
          className="ml-2 rounded border border-gray-300 px-3 py-1 shadow-md outline-none transition focus:border-purple-700 focus:ring-1 focus:ring-[#6805F2]"
          value={
            tartarus.bool_controller
              ? 'controller'
              : tartarus.debug_mode
                ? 'debug'
                : 'hades'
          }
          onChange={(e) => {
            const value = e.target.value;

            if (value === 'controller') {
              toggleBoolean('bool_controller', tartarus.bool_controller);
              if (tartarus.debug_mode)
                toggleBoolean('debug_mode', tartarus.debug_mode);
            } else if (value === 'debug') {
              toggleBoolean('debug_mode', tartarus.debug_mode);
              if (tartarus.bool_controller)
                toggleBoolean('bool_controller', tartarus.bool_controller);
            } else {
              if (tartarus.bool_controller)
                toggleBoolean('bool_controller', tartarus.bool_controller);
              if (tartarus.debug_mode)
                toggleBoolean('debug_mode', tartarus.debug_mode);
            }
          }}
        >
          <option value="hades">Hades</option>
          <option value="controller">Modo Controller</option>
          <option value="debug">Modo Debug</option>
        </select>
      </RowWrapper>

      <RowWrapper title="Time Azul:">
        <span className="font-mono">{tartarus.team_blue ? 'Sim' : 'Não'}</span>
      </RowWrapper>

      <GoalkeeperIdInput />

      <h2 className="text-lg font-bold mb-4">Portas</h2>

      <NumberInputRow
        label="STM Port:"
        value={stmPort}
        setValue={setStmPort}
        onSubmit={() => updateNumber('stm_port', stmPort)}
      />
      <NumberInputRow
        label="GC Port:"
        value={mcastGCPort}
        setValue={setMcastPortGC}
        onSubmit={() => updateNumber('mcast_port_gc', mcastGCPort)}
      />
      <NumberInputRow
        label="SSL Vision Port:"
        value={mcastSslVisionPort}
        setValue={setSslVisionPort}
        onSubmit={() =>
          updateNumber('mcast_port_vision_sslvision', mcastSslVisionPort)
        }
      />
      <NumberInputRow
        label="GrSim Port:"
        value={mcastGrsimPort}
        setValue={setMcastGrsimPort}
        onSubmit={() => updateNumber('mcast_port_vision_grsim', mcastGrsimPort)}
      />
      <NumberInputRow
        label="AutoReferee Port:"
        value={mcastTrackedPort}
        setValue={setTrackedPort}
        onSubmit={() =>
          updateNumber('mcast_port_vision_tracked', mcastTrackedPort)
        }
      />

      <h2 className="text-lg font-bold mb-4">Campo</h2>

      <RowWrapper title="Orientação do Campo:">
        <ActionButton
          onClick={async () => {
            const newFlip = !flipField;
            setFlipField(newFlip);

            if (tartarus.half_field) {
              const payload = { right_field: newFlip };
              const success = await sendPost(
                'http://localhost:5000/data',
                payload,
              );
              if (!success) console.error('Erro ao enviar left_field');
              else console.log('left_field enviado:', newFlip);
            }
          }}
          label={flipField ? 'Normal' : 'Inverter'}
        />
      </RowWrapper>

      <RowWrapper title="Dimensões do Campo:">
        <ActionButton
          onClick={() => setReceptDimensions(!receptDimensions)}
          label={receptDimensions ? 'Fixas' : 'SSL-Vision'}
        />
      </RowWrapper>

      <NumberInputRow
        label="Número de Cameras:"
        value={camsNumber}
        setValue={setCamsNumber}
        onSubmit={() => updateNumber('cams_number', camsNumber)}
      />

      <RowWrapper title="Meio Campo:">
        <ToggleSwitch
          value={value}
          onToggle={() => toggleBoolean('half_field', tartarus.half_field)}
        />
      </RowWrapper>
    </div>
  );
}
