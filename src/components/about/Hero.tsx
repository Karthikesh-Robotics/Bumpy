import React from 'react';
import { Bot } from 'lucide-react';
import { motion } from 'framer-motion';

const Hero: React.FC = () => {
  return (
    <section className="relative h-[70vh] flex items-center justify-center overflow-hidden">
      {/* Background with gradient overlay */}
      <div className="absolute inset-0 bg-gradient-to-r from-black/90 to-black/60 z-10"></div>
      
      {/* Background image */}
      <div className="absolute inset-0 bg-[url('https://images.pexels.com/photos/3912379/pexels-photo-3912379.jpeg')] bg-cover bg-center"></div>
      
      {/* Content */}
      <motion.div 
        className="container mx-auto px-4 relative z-20 text-center"
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.6 }}
      >
        <motion.div 
          className="flex justify-center mb-6"
          whileHover={{ scale: 1.1 }}
          transition={{ type: "spring", stiffness: 400, damping: 10 }}
        >
          <div className="bg-green-500 p-4 rounded-full">
            <Bot className="h-12 w-12 text-black" />
          </div>
        </motion.div>
        <motion.div className="max-w-4xl mx-auto">
          <motion.h1 
            className="text-3xl md:text-6xl font-bold text-white mb-6"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2, duration: 0.6 }}
          >
            About <span className="text-green-500">Karthikesh Robotics</span>
          </motion.h1>
          <motion.p 
            className="text-lg md:text-xl text-gray-200 max-w-3xl mx-auto"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.4, duration: 0.6 }}
          >
            Pioneering the future of autonomous robotics with innovative solutions
            that transform industries and educate the next generation.
          </motion.p>
        </motion.div>
      </motion.div>
      
      {/* Animated scroll indicator */}
      <motion.div 
        className="absolute bottom-8 left-1/2 transform -translate-x-1/2 z-20"
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ delay: 0.8, duration: 0.6 }}
      >
        <div className="w-8 h-12 border-2 border-white rounded-full flex justify-center">
          <div className="w-2 h-2 bg-white rounded-full animate-scroll-down"></div>
        </div>
      </motion.div>
      
      {/* Custom animation */}
      <style jsx>{`
        @keyframes scrollDown {
          0% {
            transform: translateY(0);
            opacity: 1;
          }
          75% {
            transform: translateY(24px);
            opacity: 0;
          }
          80% {
            transform: translateY(0);
            opacity: 0;
          }
          100% {
            opacity: 1;
          }
        }
        .animate-scroll-down {
          animation: scrollDown 2s infinite;
        }
      `}</style>
    </section>
  );
};

export default Hero;