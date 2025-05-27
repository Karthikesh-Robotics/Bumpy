import React from 'react';
import { ExternalLink } from 'lucide-react';

interface PortfolioItemProps {
  title: string;
  category: string;
  image: string;
  description: string;
}

const PortfolioItem: React.FC<PortfolioItemProps> = ({
  title,
  category,
  image,
  description,
}) => {
  return (
    <div className="group relative overflow-hidden rounded-lg shadow-md bg-white">
      <div className="aspect-video overflow-hidden">
        <img
          src={image}
          alt={title}
          className="w-full h-full object-cover transition-transform duration-500 group-hover:scale-110"
        />
      </div>
      
      <div className="absolute inset-0 bg-gradient-to-t from-black/80 via-black/50 to-transparent opacity-0 group-hover:opacity-100 transition-all duration-300 flex flex-col justify-end p-6">
        <span className="text-green-400 text-sm font-medium mb-2 transform translate-y-4 group-hover:translate-y-0 transition-transform duration-300">
          {category}
        </span>
        <h3 className="text-white text-xl font-semibold mb-2 transform translate-y-4 group-hover:translate-y-0 transition-transform duration-300 delay-75">
          {title}
        </h3>
        <p className="text-gray-200 text-sm mb-4 transform translate-y-4 group-hover:translate-y-0 transition-transform duration-300 delay-100">
          {description}
        </p>
        <button className="self-start bg-green-500 hover:bg-green-600 text-black font-medium py-2 px-4 rounded-md flex items-center gap-2 transform translate-y-4 group-hover:translate-y-0 transition-transform duration-300 delay-150">
          View Project
          <ExternalLink className="h-4 w-4" />
        </button>
      </div>
    </div>
  );
};

export default PortfolioItem;