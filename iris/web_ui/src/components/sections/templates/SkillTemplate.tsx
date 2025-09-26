import SkillButton from '../utilities/SkillButton';

export default function SkillTemplate() {
  return (
    <>
      <div className='grid grid-cols-3 gap-1'>
        <SkillButton label="TOUCH LINE" value={6} color="default" />
      </div>
    </>
  );
}
