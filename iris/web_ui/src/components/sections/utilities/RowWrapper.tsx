type Props = {
  children: React.ReactNode;
  className?: string;
  title?: string;
};

export function RowWrapper({ children, className, title }: Props) {
  return (
    <div
      className={`flex items-center justify-between mb-2 ${className} group`}
    >
      {title && (
        <p className="transition-colors group-hover:text-purple-600">{title}</p>
      )}
      {children}
    </div>
  );
}
