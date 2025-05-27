import React from 'react';
import { motion } from 'framer-motion';

// Removed the 8th logo (cli8.jpg) from the array
const clientLogos = [
  '/assets/clients/cli1.png',
  '/assets/clients/cli2.jpg',
  '/assets/clients/cli3.png',
  '/assets/clients/cli4.png',
  '/assets/clients/cli5.png',
  '/assets/clients/cli6.jpg',
  '/assets/clients/cli7.png',
];

const OurClients: React.FC = () => {
  // Create two rows of logos for better control
  const firstRowLogos = clientLogos.slice(0, 4);
  const secondRowLogos = clientLogos.slice(4);

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2,
      },
    },
  };

  const itemVariants = {
    hidden: { opacity: 0, y: 20 },
    visible: {
      opacity: 1,
      y: 0,
      transition: {
        duration: 0.6,
      },
    },
  };

  return (
    <section className="py-20 bg-gray-50">
      <motion.div 
        className="container mx-auto px-8"
        variants={containerVariants}
        initial="hidden"
        whileInView="visible"
        viewport={{ once: true }}
      >
        <motion.div 
          className="text-center mb-16"
          variants={itemVariants}
        >
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Our <span className="text-green-500">Clients</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            We are proud to have worked with these esteemed clients.
          </p>
        </motion.div>

        {/* First row with 4 logos */}
        <motion.div 
          className="grid grid-cols-1 md:grid-cols-4 gap-16 items-center justify-items-center mb-16 px-4"
          variants={containerVariants}
        >
          {firstRowLogos.map((logo, index) => (
            <motion.div 
              key={index} 
              className="w-80 h-56 flex items-center justify-center p-8 rounded-xl border border-gray-200 shadow-sm hover:shadow-md transition-shadow duration-300"
              variants={itemVariants}
              whileHover={{ scale: 1.05 }}
            >
              <img
                src={logo}
                alt={`Client ${index + 1}`}
                className="max-h-full max-w-full object-contain"
                loading="lazy"
              />
            </motion.div>
          ))}
        </motion.div>
        
        {/* Second row with 3 logos centered */}
        <motion.div 
          className="flex flex-col md:flex-row justify-center items-center gap-12"
          variants={containerVariants}
        >
          {secondRowLogos.map((logo, index) => (
            <motion.div 
              key={index} 
              className="w-80 h-56 flex items-center justify-center p-8 rounded-xl border border-gray-200 shadow-sm hover:shadow-md transition-shadow duration-300"
              variants={itemVariants}
              whileHover={{ scale: 1.05 }}
            >
              <img
                src={logo}
                alt={`Client ${index + 5}`}
                className="max-h-full max-w-full object-contain"
                loading="lazy"
              />
            </motion.div>
          ))}
        </motion.div>
      </motion.div>
    </section>
  );
};

export default OurClients;