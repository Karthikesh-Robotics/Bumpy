import React from 'react';
import { Bot, BookOpen, School, Cog, Code, ShipIcon as ChipIcon } from 'lucide-react';
import { motion } from 'framer-motion';

const ServicesPage: React.FC = () => {
  const services = [
    {
      icon: <Bot className="h-12 w-12 text-green-500" />,
      title: 'Autonomous Mobile Robots',
      description: 'Custom-designed autonomous robots for industrial and educational applications, featuring advanced navigation and sensing capabilities.',
      features: [
        'Path Planning & Navigation',
        'Obstacle Avoidance',
        'Multi-Robot Coordination',
        'Custom Hardware Integration'
      ]
    },
    {
      icon: <BookOpen className="h-12 w-12 text-green-500" />,
      title: 'ROS2 Development & Training',
      description: 'Comprehensive ROS2 development services and training programs for organizations and individuals.',
      features: [
        'Custom ROS2 Package Development',
        'System Integration',
        'Performance Optimization',
        'Hands-on Training Workshops'
      ]
    },
    {
      icon: <School className="h-12 w-12 text-green-500" />,
      title: 'Educational Programs',
      description: 'Structured learning programs and workshops for students and professionals in robotics and automation.',
      features: [
        'Robotics Fundamentals',
        'Programming Skills',
        'Hardware Integration',
        'Project-Based Learning'
      ]
    },
    {
      icon: <Cog className="h-12 w-12 text-green-500" />,
      title: 'Lab Setup & Consultation',
      description: 'End-to-end solutions for setting up robotics laboratories and research facilities.',
      features: [
        'Requirements Analysis',
        'Equipment Selection',
        'Installation & Configuration',
        'Ongoing Support'
      ]
    },
    {
      icon: <Code className="h-12 w-12 text-green-500" />,
      title: 'Custom Software Development',
      description: 'Tailored software solutions for robotics applications and automation systems.',
      features: [
        'Control Systems',
        'User Interfaces',
        'Data Analytics',
        'System Integration'
      ]
    },
    {
      icon: <ChipIcon className="h-12 w-12 text-green-500" />,
      title: 'Hardware Integration',
      description: 'Seamless integration of sensors, actuators, and control systems for robotic applications.',
      features: [
        'Sensor Integration',
        'Motor Control',
        'Power Management',
        'Custom PCB Design'
      ]
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
    <div className="pt-20">
      <section className="relative py-20 bg-white">
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
            <h1 className="text-4xl md:text-5xl font-bold mb-4">
              Our <span className="text-green-500">Services</span>
            </h1>
            <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
            <p className="text-xl text-gray-600 max-w-3xl mx-auto">
              Comprehensive robotics solutions tailored to your needs, from development to education
            </p>
          </motion.div>

          <motion.div 
            className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8"
            variants={containerVariants}
          >
            {services.map((service, index) => (
              <motion.div 
                key={index}
                className="bg-white p-8 rounded-xl shadow-lg hover:shadow-xl transition-all duration-300 border border-gray-100 hover:border-green-500"
                variants={itemVariants}
                whileHover={{ 
                  scale: 1.02,
                  transition: { type: "spring", stiffness: 300 }
                }}
              >
                <motion.div 
                  className="mb-6"
                  whileHover={{ 
                    rotate: 360,
                    transition: { duration: 0.6 }
                  }}
                >
                  {service.icon}
                </motion.div>
                <h3 className="text-2xl font-semibold mb-4">{service.title}</h3>
                <p className="text-gray-600 mb-6">{service.description}</p>
                <motion.ul 
                  className="space-y-3"
                  variants={containerVariants}
                >
                  {service.features.map((feature, idx) => (
                    <motion.li 
                      key={idx} 
                      className="flex items-center gap-3"
                      variants={itemVariants}
                      whileHover={{ x: 10 }}
                    >
                      <div className="h-2 w-2 bg-green-500 rounded-full"></div>
                      <span className="text-gray-700">{feature}</span>
                    </motion.li>
                  ))}
                </motion.ul>
              </motion.div>
            ))}
          </motion.div>
        </motion.div>
      </section>
    </div>
  );
};

export default ServicesPage;