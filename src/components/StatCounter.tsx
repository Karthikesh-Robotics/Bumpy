import React, { useState, useEffect } from 'react';

interface StatItemProps {
  number: number;
  label: string;
  duration?: number;
  delay?: number;
}

const StatItem: React.FC<StatItemProps> = ({ 
  number, 
  label, 
  duration = 2000, 
  delay = 0 
}) => {
  const [count, setCount] = useState(0);
  
  useEffect(() => {
    let startTimestamp: number | null = null;
    let animationFrameId: number;
    
    // Delay start of animation
    const timeoutId = setTimeout(() => {
      const step = (timestamp: number) => {
        if (!startTimestamp) startTimestamp = timestamp;
        const progress = timestamp - startTimestamp;
        const percentage = Math.min(progress / duration, 1);
        
        // Easing function for smoother animation
        const easeOutCubic = 1 - Math.pow(1 - percentage, 3);
        
        setCount(Math.floor(easeOutCubic * number));
        
        if (percentage < 1) {
          animationFrameId = requestAnimationFrame(step);
        }
      };
      
      animationFrameId = requestAnimationFrame(step);
    }, delay);
    
    return () => {
      clearTimeout(timeoutId);
      cancelAnimationFrame(animationFrameId);
    };
  }, [number, duration, delay]);
  
  return (
    <div className="flex flex-col items-center">
      <div className="flex items-center gap-2">
        <span className="text-4xl font-bold text-gray-900">{count}</span>
        <svg 
          className="w-10 h-10 text-green-500" 
          viewBox="0 0 24 24" 
          fill="none" 
          xmlns="http://www.w3.org/2000/svg"
        >
          <path 
            d="M12 6V12M12 12V18M12 12H18M12 12H6" 
            stroke="currentColor" 
            strokeWidth="2" 
            strokeLinecap="round" 
            strokeLinejoin="round"
          />
        </svg>
      </div>
      <span className="text-lg font-medium uppercase tracking-wider text-gray-900">{label}</span>
    </div>
  );
};

const StatCounter: React.FC = () => {
  return (
    <div className="w-full py-8 border-t border-gray-200">
      <div className="container mx-auto">
        <div className="grid grid-cols-2 md:grid-cols-4 gap-8">
          <StatItem number={7} label="Robots Delivered" delay={0} />
          <StatItem number={22} label="Workshops" delay={300} />
          <StatItem number={256} label="Interns" delay={600} />
          <StatItem number={1091} label="Students Mentored" delay={900} />
        </div>
      </div>
    </div>
  );
};

export default StatCounter;