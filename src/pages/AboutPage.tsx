import React from 'react';
import { motion } from 'framer-motion';
import Hero from '../components/about/Hero';
import Timeline from '../components/about/Timeline';
import MissionVision from '../components/about/MissionVision';
import Values from '../components/about/Values';
import WhyChooseUs from '../components/about/WhyChooseUs';
import Team from '../components/about/Team';
import LocationMap from '../components/about/LocationMap';

const AboutPage: React.FC = () => {
  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.3,
      },
    },
  };

  const sectionVariants = {
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
    <motion.div 
      className="bg-white"
      variants={containerVariants}
      initial="hidden"
      animate="visible"
    >
      <motion.div variants={sectionVariants}>
        <Hero />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <Timeline />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <MissionVision />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <Values />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <WhyChooseUs />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <Team />
      </motion.div>
      <motion.div variants={sectionVariants}>
        <LocationMap />
      </motion.div>
    </motion.div>
  );
};

export default AboutPage;