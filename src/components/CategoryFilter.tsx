import React from 'react';
import { CheckCircle2 } from 'lucide-react';

interface CategoryFilterProps {
  categories: string[];
  selectedCategory: string | null;
  onSelectCategory: (category: string | null) => void;
}

const CategoryFilter: React.FC<CategoryFilterProps> = ({ 
  categories, 
  selectedCategory, 
  onSelectCategory 
}) => {
  const getCategoryColor = (category: string) => {
    const categoryColors: Record<string, string> = {
      Robotics: 'bg-blue-500 text-white',
      AIML: 'bg-purple-500 text-white',
      'GEN AI': 'bg-indigo-500 text-white',
      RoboDK: 'bg-orange-500 text-white',
      ROS: 'bg-green-500 text-white',
      'AR VR': 'bg-red-500 text-white',
      'Project Mentoring': 'bg-yellow-500 text-white',
    };

    // Check if the category contains any of the keys
    for (const key in categoryColors) {
      if (category.includes(key)) {
        return categoryColors[key];
      }
    }

    return 'bg-gray-500 text-white';
  };

  const getHoverColor = (category: string) => {
    const categoryHoverColors: Record<string, string> = {
      Robotics: 'hover:bg-blue-600',
      AIML: 'hover:bg-purple-600',
      'GEN AI': 'hover:bg-indigo-600',
      RoboDK: 'hover:bg-orange-600',
      ROS: 'hover:bg-green-600',
      'AR VR': 'hover:bg-red-600',
      'Project Mentoring': 'hover:bg-yellow-600',
    };

    // Check if the category contains any of the keys
    for (const key in categoryHoverColors) {
      if (category.includes(key)) {
        return categoryHoverColors[key];
      }
    }

    return 'hover:bg-gray-600';
  };

  return (
    <div className="flex flex-wrap items-center gap-2 mb-8">
      <button
        onClick={() => onSelectCategory(null)}
        className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 flex items-center gap-2
          ${selectedCategory === null 
            ? 'bg-green-500 text-white shadow-md' 
            : 'bg-gray-100 hover:bg-gray-200 text-gray-700'
          }`}
      >
        {selectedCategory === null && <CheckCircle2 className="h-4 w-4" />}
        All Categories
      </button>
      
      {categories.map(category => (
        <button
          key={category}
          onClick={() => onSelectCategory(category)}
          className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 flex items-center gap-2
            ${selectedCategory === category 
              ? `${getCategoryColor(category)} shadow-md` 
              : `bg-gray-100 hover:bg-gray-200 text-gray-700 ${getHoverColor(category)}`
            }`}
        >
          {selectedCategory === category && <CheckCircle2 className="h-4 w-4" />}
          {category}
        </button>
      ))}
    </div>
  );
};

export default CategoryFilter;