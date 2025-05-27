import React from 'react';

interface ServiceCardProps {
  icon: React.ReactNode;
  title: string;
  description: string;
  delay: number;
}

const ServiceCard: React.FC<ServiceCardProps> = ({
  icon,
  title,
  description,
  delay,
}) => {
  return (
    <div 
      className="bg-white p-8 rounded-lg shadow-md hover:shadow-xl transition-all duration-300 transform hover:-translate-y-2 border-t-4 border-green-500 group"
      style={{
        animationDelay: `${delay}s`,
      }}
    >
      <div className="text-center">
        <div className="mb-4 flex justify-center items-center">
          {icon}
        </div>
        <h3 className="text-xl font-semibold mb-3 group-hover:text-green-500 transition-colors duration-300">
          {title}
        </h3>
        <p className="text-gray-600">
          {description}
        </p>
      </div>
    </div>
  );
};

export default ServiceCard;