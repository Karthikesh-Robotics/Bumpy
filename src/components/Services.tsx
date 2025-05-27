import React from 'react';
import { Bot, BookOpen, School, Cog } from 'lucide-react';
import ServiceCard from './ServiceCard';
import { motion } from 'framer-motion';

const Services: React.FC = () => {
  const services = [
    {
      icon: <Bot className="h-8 w-8 text-green-500 mb-4" />,
      title: 'Autonomous Mobile Robots',
      description:
        'Design and development of cutting-edge autonomous mobile robots for various industrial and educational applications.',
    },
    {
      icon: <BookOpen className="h-8 w-8 text-green-500 mb-4" />,
      title: 'ROS2 Mentoring',
      description:
        'Expert guidance and mentorship in Robot Operating System 2 (ROS2) for students, researchers, and professionals.',
    },
    {
      icon: <School className="h-8 w-8 text-green-500 mb-4" />,
      title: 'Educational Workshops',
      description:
        'Interactive workshops for schools and colleges to introduce students to the exciting world of robotics and AI.',
    },
    {
      icon: <Cog className="h-8 w-8 text-green-500 mb-4" />,
      title: 'Robotics Lab Setup',
      description:
        'Complete solutions for setting up state-of-the-art robotics laboratories in educational institutions and research centers.',
    },
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
    <section id="services" className="py-20 bg-gray-50">
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
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Our <span className="text-green-500">Services</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Providing comprehensive robotics solutions from design and development to education and implementation.
          </p>
        </motion.div>

        <motion.div 
          className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8"
          variants={containerVariants}
        >
          {services.map((service, index) => (
            <motion.div
              key={index}
              variants={itemVariants}
              whileHover={{ scale: 1.05 }}
              transition={{ type: "spring", stiffness: 300 }}
            >
              <ServiceCard 
                icon={service.icon}
                title={service.title}
                description={service.description}
                delay={index * 0.1}
              />
            </motion.div>
          ))}
        </motion.div>
      </motion.div>
    </section>
  );
};

export default Services;