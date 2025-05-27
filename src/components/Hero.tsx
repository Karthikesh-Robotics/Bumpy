import React, { useEffect, useRef } from 'react';
import { ArrowRight } from 'lucide-react';
import { useNavigate, Link } from 'react-router-dom';
import { motion, useScroll, useTransform } from 'framer-motion';
import StatCounter from './StatCounter';

const Hero: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const navigate = useNavigate();
  const { scrollY } = useScroll();
  const y = useTransform(scrollY, [0, 500], [0, 100]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;

    const particles = [];

    for (let i = 0; i < 150; i++) {
      particles.push({
        x: Math.random() * canvas.width,
        y: Math.random() * canvas.height,
        size: Math.random() * 5 + 3,
        speedX: (Math.random() - 0.5) * 2,
        speedY: (Math.random() - 0.5) * 2,
        color: '#10b981',
        alpha: Math.random() * 0.6 + 0.4,
        growing: Math.random() > 0.5,
      });
    }

    const animate = () => {
      ctx.fillStyle = 'rgba(255, 255, 255, 0.1)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      particles.forEach((particle) => {
        if (particle.growing) {
          particle.alpha += 0.02;
          if (particle.alpha >= 0.9) particle.growing = false;
        } else {
          particle.alpha -= 0.02;
          if (particle.alpha <= 0.4) particle.growing = true;
        }

        ctx.beginPath();
        ctx.arc(particle.x, particle.y, particle.size, 0, Math.PI * 2);
        ctx.fillStyle = `${particle.color}${Math.floor(particle.alpha * 255)
          .toString(16)
          .padStart(2, '0')}`;
        ctx.fill();

        particles.forEach((otherParticle) => {
          const dx = particle.x - otherParticle.x;
          const dy = particle.y - otherParticle.y;
          const distance = Math.sqrt(dx * dx + dy * dy);

          if (distance < 150) {
            ctx.beginPath();
            ctx.strokeStyle = `${particle.color}${Math.floor(
              0.25 * (1 - distance / 150) * 255
            )
              .toString(16)
              .padStart(2, '0')}`;
            ctx.lineWidth = 1.2;
            ctx.moveTo(particle.x, particle.y);
            ctx.lineTo(otherParticle.x, otherParticle.y);
            ctx.stroke();
          }
        });

        particle.x += particle.speedX;
        particle.y += particle.speedY;

        if (particle.x + particle.size > canvas.width) {
          particle.x = canvas.width - particle.size;
          particle.speedX *= -1;
        }
        if (particle.x - particle.size < 0) {
          particle.x = particle.size;
          particle.speedX *= -1;
        }
        if (particle.y + particle.size > canvas.height) {
          particle.y = canvas.height - particle.size;
          particle.speedY *= -1;
        }
        if (particle.y - particle.size < 0) {
          particle.y = particle.size;
          particle.speedY *= -1;
        }
      });

      requestAnimationFrame(animate);
    };

    animate();

    const handleResize = () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

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
    <section
      id="home"
      className="relative min-h-screen flex flex-col items-center justify-center overflow-hidden"
    >
      <canvas
        ref={canvasRef}
        className="absolute inset-0 w-full h-full"
      ></canvas>
      <div className="absolute inset-0 bg-gradient-to-b from-white/80 to-white/90"></div>

      <motion.div 
        className="container mx-auto px-4 md:px-8 lg:px-20 z-20 flex-grow"
        variants={containerVariants}
        initial="hidden"
        animate="visible"
      >
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-8 items-center py-24">
          <motion.div 
            className="text-center lg:text-left lg:col-span-5"
            variants={itemVariants}
          >
            <motion.h1 
              className="text-4xl md:text-5xl lg:text-6xl font-bold text-gray-900 mb-6 leading-tight"
              variants={itemVariants}
            >
              <motion.span 
                className="block mb-2"
                variants={itemVariants}
              >
                Pioneering the Future of
              </motion.span>
              <motion.span 
                className="text-green-500 relative inline-block"
                variants={itemVariants}
              >
                Autonomous Robotics
                <span className="absolute bottom-0 left-0 w-full h-1 bg-green-500 transform scale-x-0 origin-left transition-transform duration-1000 animate-expand"></span>
              </motion.span>
            </motion.h1>

            <motion.p 
              className="text-lg md:text-xl text-gray-600 mb-8"
              variants={itemVariants}
            >
              Mentoring in ROS2, Physical AI, and building the next generation
              of robotic solutions for tomorrow's challenges.
            </motion.p>

            <motion.div 
              className="flex flex-col sm:flex-row justify-center lg:justify-start gap-4"
              variants={itemVariants}
            >
              <motion.button
                onClick={() => navigate('/services')}
                className="bg-green-500 hover:bg-green-600 text-gray-900 font-semibold py-3 px-6 rounded-md transition-all duration-300 transform hover:scale-105 hover:rotate-1 flex items-center justify-center gap-2 group h-[50px]"
                whileHover={{ scale: 1.05, rotate: 1 }}
                whileTap={{ scale: 0.95 }}
              >
                Explore Our Services
                <ArrowRight className="h-5 w-5 transform group-hover:translate-x-1 transition-transform" />
              </motion.button>
              <motion.div
                whileHover={{ scale: 1.05, rotate: -1 }}
                whileTap={{ scale: 0.95 }}
              >
                <Link
                  to="/contact"
                  className="border-2 border-green-500 text-green-500 hover:text-gray-900 hover:bg-green-500 font-semibold py-3 px-6 rounded-md transition-all duration-300 transform hover:scale-105 hover:-rotate-1 flex items-center justify-center h-[50px]"
                >
                  Get In Touch
                </Link>
              </motion.div>
            </motion.div>
          </motion.div>

          <motion.div 
            className="lg:col-span-7 relative"
            variants={itemVariants}
            style={{ y }}
          >
            <motion.div
              className="flex justify-center"
              whileHover={{ scale: 1.05, rotate: 5 }}
              transition={{ type: "spring", stiffness: 300 }}
            >
              <motion.img
                src="/assets/bumpy.png"
                alt="Robot"
                className="w-full max-w-[800px] xl:max-w-[1000px] h-auto object-contain"
                animate={{
                  y: [0, -20, 0],
                  rotate: [0, 5, -5, 0],
                }}
                transition={{
                  duration: 5,
                  repeat: Infinity,
                  ease: "easeInOut",
                }}
              />
            </motion.div>
          </motion.div>
        </div>
      </motion.div>

      <motion.div 
        className="w-full z-10 mt-auto"
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.6, delay: 0.6 }}
      >
        <StatCounter />
      </motion.div>

      <style>{`
        @keyframes float {
          0%, 100% { transform: translateY(0px); }
          50% { transform: translateY(-20px); }
        }
      `}</style>
    </section>
  );
};

export default Hero;