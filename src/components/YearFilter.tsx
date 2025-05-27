import React from 'react';
import { CheckCircle2 } from 'lucide-react';

interface YearFilterProps {
  years: number[];
  selectedYear: number | null;
  onSelectYear: (year: number | null) => void;
}

const YearFilter: React.FC<YearFilterProps> = ({ years, selectedYear, onSelectYear }) => {
  return (
    <div className="flex flex-wrap items-center gap-2 mb-8">
      <button
        onClick={() => onSelectYear(null)}
        className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 flex items-center gap-2
          ${selectedYear === null 
            ? 'bg-green-500 text-white shadow-md' 
            : 'bg-gray-100 hover:bg-gray-200 text-gray-700'
          }`}
      >
        {selectedYear === null && <CheckCircle2 className="h-4 w-4" />}
        All Years
      </button>
      
      {years.map(year => (
        <button
          key={year}
          onClick={() => onSelectYear(year)}
          className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 flex items-center gap-2
            ${selectedYear === year 
              ? 'bg-green-500 text-white shadow-md' 
              : 'bg-gray-100 hover:bg-gray-200 text-gray-700'
            }`}
        >
          {selectedYear === year && <CheckCircle2 className="h-4 w-4" />}
          {year}
        </button>
      ))}
    </div>
  );
};

export default YearFilter;