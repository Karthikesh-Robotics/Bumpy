import React from 'react';
import { CheckCircle2 } from 'lucide-react';
import { Link } from 'react-router-dom';
import { motion } from 'framer-motion';

const About: React.FC = () => {
  const strengths = [
    'Expertise in ROS2 and autonomous navigation',
    'Cutting-edge physical AI implementation',
    'Custom robotics solutions for diverse applications',
    'Experienced team of robotics engineers and AI specialists',
    'Proven track record in educational institutions',
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
    hidden: { opacity: 0, x: -20 },
    visible: {
      opacity: 1,
      x: 0,
      transition: {
        duration: 0.6,
      },
    },
  };

  return (
    <section id="about" className="py-20 bg-black text-white">
      <motion.div 
        className="container mx-auto px-4"
        variants={containerVariants}
        initial="hidden"
        whileInView="visible"
        viewport={{ once: true }}
      >
        <div className="flex flex-col lg:flex-row items-center gap-12">
          <motion.div 
            className="lg:w-1/2"
            variants={itemVariants}
          >
            <div className="relative">
              <motion.div 
                className="bg-green-500/20 rounded-lg overflow-hidden flex justify-center items-center h-64 animate-float"
                whileHover={{ scale: 1.05 }}
                transition={{ type: "spring", stiffness: 300 }}
              >
                <motion.img
                  src="/assets/logo.png"
                  alt="Karthikesh Robotics Logo"
                  className="max-h-full object-contain p-4 transition-transform duration-500"
                  whileHover={{ rotate: 5 }}
                />
              </motion.div>
              <motion.div 
                className="absolute -bottom-6 -right-6 w-32 h-32 bg-green-500 rounded-lg hidden md:block"
                animate={{
                  rotate: [0, 180, 360],
                  scale: [1, 1.1, 1],
                }}
                transition={{
                  duration: 20,
                  repeat: Infinity,
                  ease: "linear",
                }}
              />
              <motion.div 
                className="absolute -top-6 -left-6 w-32 h-32 border-2 border-green-500 rounded-lg hidden md:block"
                animate={{
                  rotate: [360, 180, 0],
                  scale: [1, 1.1, 1],
                }}
                transition={{
                  duration: 20,
                  repeat: Infinity,
                  ease: "linear",
                }}
              />
            </div>
          </motion.div>

          <motion.div 
            className="lg:w-1/2"
            variants={containerVariants}
          >
            <motion.h2 
              className="text-3xl md:text-4xl font-bold mb-6"
              variants={itemVariants}
            >
              About <span className="text-green-500">Karthikesh Robotics</span>
            </motion.h2>

            <motion.p 
              className="text-gray-300 mb-6"
              variants={itemVariants}
            >
              Karthikesh Robotics is a pioneering company at the forefront of autonomous mobile robotics technology. We combine expertise in robotics engineering, artificial intelligence, and educational outreach to deliver cutting-edge solutions and knowledge transfer.
            </motion.p>

            <motion.p 
              className="text-gray-300 mb-8"
              variants={itemVariants}
            >
              Our mission is to advance the field of robotics through innovation, education, and implementation. We work closely with educational institutions to foster the next generation of robotics engineers and researchers.
            </motion.p>

            <motion.div 
              className="space-y-3 mb-8"
              variants={containerVariants}
            >
              {strengths.map((strength, index) => (
                <motion.div 
                  key={index} 
                  className="flex items-start gap-3"
                  variants={itemVariants}
                  whileHover={{ x: 10 }}
                >
                  <CheckCircle2 className="h-6 w-6 text-green-500 flex-shrink-0 mt-0.5" />
                  <span>{strength}</span>
                </motion.div>
              ))}
            </motion.div>

            <motion.div
              variants={itemVariants}
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
            >
              <Link 
                to="/about"
                className="bg-green-500 hover:bg-green-600 text-black font-semibold py-3 px-8 rounded-md transition-all duration-300"
              >
                Learn More
              </Link>
            </motion.div>
          </motion.div>
        </div>
      </motion.div>

      <style>{`
        @keyframes float {
          0%, 100% {
            transform: translateY(0);
          }
          50% {
            transform: translateY(-10px);
          }
        }
        .animate-float {
          animation: float 4s ease-in-out infinite;
        }
      `}</style>
    </section>
  );
};

export default About;