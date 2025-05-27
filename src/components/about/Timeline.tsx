import React from 'react';
import { motion } from 'framer-motion';

const Timeline: React.FC = () => {
  const milestones = [
    {
      year: 2017,
      title: 'YouTube Channel Launch',
      description: 'Started sharing robotics knowledge and tutorials on YouTube'
    },
    {
      year: 2020,
      title: 'Freelancing Success',
      description: 'Built and delivered over 100+ robotics projects as a freelancer'
    },
    {
      year: 2024,
      title: 'Company Incorporation',
      description: 'Officially incorporated as Karthikesh Robotics Private Limited on March 22nd'
    }
  ];

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
    <section className="py-20 bg-white">
      <motion.div 
        className="container mx-auto px-4"
        variants={containerVariants}
        initial="hidden"
        whileInView="visible"
        viewport={{ once: true }}
      >
        <motion.div 
          className="text-center mb-16"
          variants={itemVariants}
        >
          <h2 className="text-3xl md:text-5xl font-bold text-black">
            Our <span className="text-green-500">Journey</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </motion.div>

        <div className="relative">
          {/* Vertical Line - Hidden on mobile */}
          <div className="absolute left-1/2 transform -translate-x-1/2 h-full w-1 bg-green-500 hidden md:block"></div>

          <div className="space-y-12 md:space-y-24">
            {milestones.map((milestone, index) => (
              <motion.div
                key={milestone.year}
                variants={itemVariants}
                className={`flex flex-col md:flex-row items-center ${
                  index % 2 === 0 ? 'md:flex-row' : 'md:flex-row-reverse'
                }`}
              >
                <div className={`w-full md:w-1/2 ${index % 2 === 0 ? 'md:pr-12' : 'md:pl-12'}`}>
                  <motion.div 
                    className="bg-gray-50 p-8 rounded-xl shadow-lg hover:shadow-xl transition-shadow"
                    whileHover={{ scale: 1.02 }}
                  >
                    <span className="text-green-500 text-4xl font-bold">{milestone.year}</span>
                    <h3 className="text-2xl font-bold mt-4 mb-2">{milestone.title}</h3>
                    <p className="text-gray-600">{milestone.description}</p>
                  </motion.div>
                </div>

                {/* Timeline Marker - Shown on mobile and desktop */}
                <div className="my-4 md:my-0 flex items-center justify-center">
                  <motion.div 
                    className="w-8 h-8 bg-green-500 rounded-full border-4 border-white shadow-lg"
                    whileHover={{ scale: 1.2 }}
                  />
                  {/* Mobile Line */}
                  <div className="h-1 w-24 bg-green-500 block md:hidden mx-4"></div>
                </div>

                {/* Empty div for layout on desktop */}
                <div className="hidden md:block md:w-1/2"></div>
              </motion.div>
            ))}
          </div>
        </div>
      </motion.div>
    </section>
  );
};

export default Timeline;