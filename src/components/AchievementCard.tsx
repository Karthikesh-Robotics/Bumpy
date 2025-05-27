import React, { useState } from 'react';
import { Calendar, MapPin, ChevronDown, ChevronUp } from 'lucide-react';
import { Link } from 'react-router-dom';

interface AchievementCardProps {
  id: string;
  date: string;
  title: string;
  location: string;
  category: string;
  images: string[];
  expanded?: boolean;
}

const AchievementCard: React.FC<AchievementCardProps> = ({
  id,
  date,
  title,
  location,
  category,
  images,
  expanded: initialExpanded = false,
}) => {
  const [expanded, setExpanded] = useState(initialExpanded);
  const [currentImageIndex, setCurrentImageIndex] = useState(0);

  const handleNextImage = () => {
    setCurrentImageIndex((prev) => (prev + 1) % images.length);
  };

  const handlePrevImage = () => {
    setCurrentImageIndex((prev) => (prev - 1 + images.length) % images.length);
  };

  const getCategoryColor = () => {
    const categories: Record<string, string> = {
      Robotics: 'bg-blue-100 text-blue-600',
      AIML: 'bg-purple-100 text-purple-600',
      'GEN AI': 'bg-indigo-100 text-indigo-600',
      RoboDK: 'bg-orange-100 text-orange-600',
      ROS: 'bg-green-100 text-green-600',
      'AR VR': 'bg-red-100 text-red-600',
      'Project Mentoring': 'bg-yellow-100 text-yellow-600',
    };

    for (const key in categories) {
      if (category.includes(key)) {
        return categories[key];
      }
    }

    return 'bg-gray-100 text-gray-600';
  };

  return (
    <div className="bg-white rounded-xl shadow-md overflow-hidden transition-all duration-300 hover:shadow-xl">
      <div className="relative h-64 overflow-hidden">
        {images.length > 0 && (
          <>
            <img
              src={images[currentImageIndex]}
              alt={`${title} - photo ${currentImageIndex + 1}`}
              className="w-full h-full object-cover transition-transform duration-700"
            />
            
            {images.length > 1 && (
              <>
                <button
                  onClick={handlePrevImage}
                  className="absolute left-2 top-1/2 -translate-y-1/2 bg-black bg-opacity-30 hover:bg-opacity-50 text-white rounded-full p-2 transition-all duration-200"
                  aria-label="Previous image"
                >
                  <ChevronUp className="rotate-90 h-5 w-5" />
                </button>
                <button
                  onClick={handleNextImage}
                  className="absolute right-2 top-1/2 -translate-y-1/2 bg-black bg-opacity-30 hover:bg-opacity-50 text-white rounded-full p-2 transition-all duration-200"
                  aria-label="Next image"
                >
                  <ChevronDown className="rotate-270 h-5 w-5" />
                </button>
                
                <div className="absolute bottom-2 left-0 right-0 flex justify-center gap-1">
                  {images.map((_, index) => (
                    <button
                      key={index}
                      onClick={() => setCurrentImageIndex(index)}
                      className={`h-2 w-2 rounded-full transition-all ${
                        index === currentImageIndex
                          ? 'bg-white scale-125'
                          : 'bg-white bg-opacity-50'
                      }`}
                      aria-label={`Go to image ${index + 1}`}
                    />
                  ))}
                </div>
              </>
            )}
          </>
        )}
        
        <div className="absolute top-2 left-2">
          <span className={`px-3 py-1 rounded-full text-xs font-medium ${getCategoryColor()}`}>
            {category}
          </span>
        </div>
      </div>
      
      <div className="p-5">
        <div className="flex items-center gap-2 text-gray-500 text-sm mb-2">
          <Calendar className="h-4 w-4" />
          <span>{date}</span>
        </div>
        
        <h3 className="text-xl font-semibold mb-3">{title}</h3>
        
        <div className="flex items-center gap-2 text-gray-500 mb-4">
          <MapPin className="h-4 w-4" />
          <span>{location}</span>
        </div>
        
        <div className="flex justify-between items-center">
          <button
            onClick={() => setExpanded(!expanded)}
            className="flex items-center gap-2 text-gray-500 hover:text-green-500 transition-colors"
          >
            {expanded ? (
              <>
                <span>Show less</span>
                <ChevronUp className="h-4 w-4" />
              </>
            ) : (
              <>
                <span>View all photos</span>
                <ChevronDown className="h-4 w-4" />
              </>
            )}
          </button>

          <Link
            to={`/workshop/${id}`}
            className="px-4 py-2 bg-green-500 text-white rounded-full hover:bg-green-600 transition-colors"
          >
            View More
          </Link>
        </div>
        
        {expanded && (
          <div className="mt-4 grid grid-cols-2 gap-2">
            {images.map((image, index) => (
              <div 
                key={index} 
                className={`overflow-hidden rounded-md ${
                  index === 0 ? 'col-span-2 row-span-2' : ''
                }`}
              >
                <img
                  src={image}
                  alt={`${title} - gallery photo ${index + 1}`}
                  className="w-full h-full object-cover transition-transform hover:scale-105 duration-500"
                  onClick={() => setCurrentImageIndex(index)}
                />
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default AchievementCard;