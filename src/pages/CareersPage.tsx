import React from 'react';
import { motion } from 'framer-motion';
import { Briefcase, Users, Rocket, Brain, ArrowRight } from 'lucide-react';

const CareersPage: React.FC = () => {
  return (
    <div className="pt-20 min-h-screen bg-black text-white">
      <section className="py-20">
        <div className="container mx-auto px-4">
          <motion.div 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
            className="text-center mb-16"
          >
            <h1 className="text-4xl md:text-5xl font-bold mb-4">
              Join Our <span className="text-green-500">Team</span>
            </h1>
            <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
            <p className="text-xl text-gray-300 max-w-3xl mx-auto">
              Be part of the future of robotics and help shape the next generation of autonomous systems
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 mb-20">
            {[
              {
                icon: <Users className="h-12 w-12 text-green-500" />,
                title: "Collaborative Environment",
                description: "Work with passionate experts in robotics and AI"
              },
              {
                icon: <Rocket className="h-12 w-12 text-green-500" />,
                title: "Innovation Focus",
                description: "Push the boundaries of what's possible in robotics"
              },
              {
                icon: <Brain className="h-12 w-12 text-green-500" />,
                title: "Learning & Growth",
                description: "Continuous learning and development opportunities"
              },
              {
                icon: <Briefcase className="h-12 w-12 text-green-500" />,
                title: "Work-Life Balance",
                description: "Flexible work arrangements and great benefits"
              }
            ].map((benefit, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.6, delay: index * 0.1 }}
                className="bg-gray-900 p-6 rounded-xl border border-gray-800 hover:border-green-500 transition-all duration-300"
              >
                <div className="mb-4">{benefit.icon}</div>
                <h3 className="text-xl font-semibold mb-2">{benefit.title}</h3>
                <p className="text-gray-400">{benefit.description}</p>
              </motion.div>
            ))}
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.8 }}
            className="max-w-4xl mx-auto"
          >
            <div className="bg-gray-900 rounded-2xl p-8 relative overflow-hidden">
              <div className="absolute top-0 right-0 w-1/3 h-full">
                <motion.div
                  className="w-full h-full"
                  animate={{
                    background: [
                      "linear-gradient(45deg, #10b981 0%, transparent 100%)",
                      "linear-gradient(45deg, transparent 0%, #10b981 100%)",
                      "linear-gradient(45deg, #10b981 0%, transparent 100%)",
                    ],
                  }}
                  transition={{
                    duration: 5,
                    repeat: Infinity,
                    repeatType: "reverse",
                  }}
                />
              </div>

              <div className="relative z-10">
                <h2 className="text-3xl font-bold mb-6">
                  Internship <span className="text-green-500">Program</span>
                </h2>
                <p className="text-gray-300 mb-8">
                  Join our dynamic internship program and work alongside industry experts on cutting-edge robotics projects.
                  Gain hands-on experience with the latest technologies while building your career in robotics and automation.
                </p>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-8">
                  <div className="bg-black/50 p-6 rounded-xl">
                    <h3 className="text-xl font-semibold mb-4">What You'll Learn</h3>
                    <ul className="space-y-3">
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>ROS2 Development</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Autonomous Navigation</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Computer Vision</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Machine Learning</span>
                      </li>
                    </ul>
                  </div>

                  <div className="bg-black/50 p-6 rounded-xl">
                    <h3 className="text-xl font-semibold mb-4">Benefits</h3>
                    <ul className="space-y-3">
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Hands-on Project Experience</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Mentorship from Experts</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Certificate of Completion</span>
                      </li>
                      <li className="flex items-center gap-2">
                        <div className="w-2 h-2 bg-green-500 rounded-full" />
                        <span>Potential Full-time Opportunities</span>
                      </li>
                    </ul>
                  </div>
                </div>

                <a
                  href="https://forms.gle/tnDsuXXk42d5SzpE6"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="inline-flex items-center bg-green-500 hover:bg-green-600 text-black font-semibold py-3 px-8 rounded-md transition-all duration-300 group"
                >
                  Apply for Internship
                  <ArrowRight className="h-5 w-5 ml-2 transform group-hover:translate-x-1 transition-transform" />
                </a>
              </div>
            </div>
          </motion.div>
        </div>
      </section>
    </div>
  );
};

export default CareersPage;