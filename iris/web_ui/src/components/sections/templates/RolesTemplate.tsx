import RoleButton from '../utilities/RoleButton';

export default function RolesTemplate() {
  return (
    <>
      <div className='grid grid-cols-3 gap-1'>
        <RoleButton label="TOUCH LINE" value={6} color="default" />
      </div>
    </>
  );
}
