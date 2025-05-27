import React from 'react';
import { motion, useScroll, useTransform } from 'framer-motion';
import { Bot, ArrowRight, MapPin, Code, Cpu, Zap, Cog, Wrench, Battery, Wifi, Users, Network, Compass, Gauge, HardDrive, Camera, Monitor, Cherry as Raspberry, BatteryCharging, Car as SdCard, Package } from 'lucide-react';
import { useInView } from 'react-intersection-observer';

const ProductsPage: React.FC = () => {
  const { scrollY } = useScroll();
  const y = useTransform(scrollY, [0, 1000], [0, 200]);

  const [ref, inView] = useInView({
    triggerOnce: true,
    threshold: 0.1,
  });

  const includedItems = [
    { icon: <Compass className="h-8 w-8" />, name: 'IMU', description: 'High-precision inertial measurement unit' },
    { icon: <Gauge className="h-8 w-8" />, name: 'LIDAR', description: '360° laser scanning for mapping' },
    { icon: <HardDrive className="h-8 w-8" />, name: 'ENCODER', description: 'Precise position tracking' },
    { icon: <Cpu className="h-8 w-8" />, name: 'N20 MOTOR', description: 'High-torque micro gear motors' },
    { icon: <Cog className="h-8 w-8" />, name: 'MOTOR DRIVER', description: 'Advanced motor control system' },
    { icon: <Zap className="h-8 w-8" />, name: 'ALL FEATURES', description: 'Complete functionality package' },
    { icon: <Camera className="h-8 w-8" />, name: 'CAMERA', description: 'HD vision system' },
    { icon: <Monitor className="h-8 w-8" />, name: 'OLED DISPLAY', description: 'Real-time status display' },
    { icon: <Raspberry className="h-8 w-8" />, name: 'RASPBERRY PI', description: 'Powerful onboard computer' },
    { icon: <BatteryCharging className="h-8 w-8" />, name: 'BATTERY & CHARGER', description: 'Long-lasting power solution' },
    { icon: <SdCard className="h-8 w-8" />, name: 'SD CARD & READER', description: 'Expandable storage system' },
    { icon: <Package className="h-8 w-8" />, name: 'OTHER COMPONENTS', description: 'Additional accessories included' }
  ];

  const products = [
    {
      name: "BUMPY - Autonomous Mobile Robot",
      description: "Advanced autonomous mobile robot designed for educational and research purposes. Features ROS2 integration, advanced navigation, and modular design.",
      image: "/assets/bumpy.png",
      features: [
        {
          icon: <MapPin className="h-6 w-6" />,
          title: "Mapping",
          description: "Advanced SLAM for real-time mapping"
        },
        {
          icon: <Code className="h-6 w-6" />,
          title: "ROS2 Ready",
          description: "Complete development environment"
        },
        {
          icon: <Cpu className="h-6 w-6" />,
          title: "Smart Processing",
          description: "Powerful onboard computing"
        },
        {
          icon: <Zap className="h-6 w-6" />,
          title: "Fast Response",
          description: "Real-time control system"
        },
        {
          icon: <Cog className="h-6 w-6" />,
          title: "Modular Design",
          description: "Easy to customize and upgrade"
        },
        {
          icon: <Wrench className="h-6 w-6" />,
          title: "Easy Maintenance",
          description: "Accessible components"
        },
        {
          icon: <Battery className="h-6 w-6" />,
          title: "Long Battery Life",
          description: "6+ hours of operation"
        },
        {
          icon: <Wifi className="h-6 w-6" />,
          title: "Wireless Control",
          description: "Remote operation capability"
        }
      ],
      specifications: {
        dimensions: "500mm x 400mm x 300mm",
        weight: "15kg",
        battery: "24V Lithium-ion",
        runtime: "4-6 hours",
        maxSpeed: "1.5 m/s"
      }
    }
  ];

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.1,
      },
    },
  };

  const itemVariants = {
    hidden: { opacity: 0, scale: 0.8 },
    visible: {
      opacity: 1,
      scale: 1,
      transition: {
        type: "spring",
        stiffness: 100,
      },
    },
  };

  return (
    <div className="min-h-screen bg-white overflow-x-hidden">
      <section className="relative py-20">
        <div className="container mx-auto px-4">
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
          >
            <h1 className="text-4xl md:text-5xl font-bold mb-4 text-gray-900">
              Meet <span className="text-green-500">BUMPY</span>
            </h1>
            <p className="text-xl text-gray-600 max-w-3xl mx-auto">
              Your Perfect Companion for Robotics Education and Research
            </p>
          </motion.div>

          <div className="relative">
            <motion.div 
              className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-[120%] h-[120%] rounded-lg border-2 border-green-500/20"
              animate={{
                rotate: 360,
              }}
              transition={{
                duration: 20,
                repeat: Infinity,
                ease: "linear",
              }}
            />
            
            <motion.div 
              className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-[110%] h-[110%] rounded-lg border-2 border-green-500/30"
              animate={{
                rotate: -360,
              }}
              transition={{
                duration: 15,
                repeat: Infinity,
                ease: "linear",
              }}
            />

            <div className="relative z-10 max-w-3xl mx-auto">
              <motion.img
                src="/assets/bumpy.png"
                alt="BUMPY Robot"
                className="w-full h-auto"
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.6 }}
              />
            </div>

            <motion.div 
              className="grid grid-cols-2 md:grid-cols-4 gap-4 mt-12"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.3 }}
            >
              {Object.entries(products[0].specifications).map(([key, value], idx) => (
                <motion.div
                  key={idx}
                  className="bg-gray-50 p-6 rounded-lg border border-green-500/20 text-center"
                  whileHover={{ scale: 1.05, borderColor: "rgba(16, 185, 129, 0.5)" }}
                >
                  <h3 className="text-green-500 text-sm uppercase mb-2">
                    {key.replace(/([A-Z])/g, ' $1').trim()}
                  </h3>
                  <p className="text-xl font-bold text-gray-900">{value}</p>
                </motion.div>
              ))}
            </motion.div>
          </div>
        </div>
      </section>

      {/* What's Included Section */}
      <section className="py-20 bg-gray-50 overflow-hidden">
        <div className="container mx-auto px-4">
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
          >
            <h2 className="text-3xl md:text-4xl font-bold mb-4">
              What's <span className="text-green-500">Included</span>
            </h2>
            <p className="text-gray-600 max-w-3xl mx-auto">
              Everything you need to get started with your BUMPY robot
            </p>
          </motion.div>

          <motion.div 
            className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-8"
            variants={containerVariants}
            initial="hidden"
            whileInView="visible"
            viewport={{ once: true }}
          >
            {includedItems.map((item, index) => (
              <motion.div
                key={index}
                className="relative group cursor-pointer"
                variants={itemVariants}
              >
                <div className="absolute inset-0 bg-green-500 rounded-2xl opacity-0 group-hover:opacity-100 transform scale-95 group-hover:scale-100 transition-all duration-300"></div>
                <div className="relative bg-white rounded-2xl p-6 shadow-lg group-hover:shadow-xl transition-all duration-300 group-hover:translate-x-2 group-hover:-translate-y-2 z-10">
                  <div className="mb-4 text-green-500 group-hover:text-white transition-colors duration-300">
                    {item.icon}
                  </div>
                  <h3 className="font-semibold text-gray-900 group-hover:text-white transition-colors duration-300">
                    {item.name}
                  </h3>
                  <p className="text-sm text-gray-500 mt-2 group-hover:text-white/90 transition-colors duration-300">
                    {item.description}
                  </p>
                </div>
              </motion.div>
            ))}
          </motion.div>
        </div>
      </section>

      <section className="py-20 bg-white">
        <div className="container mx-auto px-4">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-12 items-start">
            <motion.div
              initial={{ opacity: 0, x: -50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.6 }}
            >
              <h2 className="text-3xl font-bold mb-6">
                Advanced <span className="text-green-500">Features</span>
              </h2>
              <div className="grid grid-cols-2 gap-4">
                {products[0].features.map((feature, idx) => (
                  <motion.div
                    key={idx}
                    className="p-4 rounded-lg bg-white hover:bg-green-50 transition-colors"
                    whileHover={{ scale: 1.05 }}
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    transition={{ delay: idx * 0.1 }}
                  >
                    <div className="text-green-500 mb-2">{feature.icon}</div>
                    <h3 className="font-semibold mb-1">{feature.title}</h3>
                    <p className="text-sm text-gray-600">{feature.description}</p>
                  </motion.div>
                ))}
              </div>
            </motion.div>

            <motion.div
              className="relative"
              initial={{ opacity: 0, x: 50 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.6 }}
            >
              <div className="w-1/2 mx-auto aspect-[9/16] rounded-lg overflow-hidden shadow-xl">
                <video 
                  className="w-full h-full object-cover"
                  controls
                  autoPlay
                  muted
                  loop
                >
                  <source src="/assets/bumpy/video2.mp4" type="video/mp4" />
                  Your browser does not support the video tag.
                </video>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      <section className="py-12 bg-white">
        <div className="container mx-auto px-4">
          <motion.div
            className="max-w-4xl mx-auto"
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
          >
            <div className="aspect-video rounded-lg overflow-hidden shadow-xl">
              <video 
                className="w-full h-full object-cover"
                controls
                autoPlay
                muted
                loop
              >
                <source src="/assets/bumpy/video1.mp4" type="video/mp4" />
                Your browser does not support the video tag.
              </video>
            </div>
          </motion.div>
        </div>
      </section>
    </div>
  );
};

export default ProductsPage;