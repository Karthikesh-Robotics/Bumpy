import React from 'react';
import { motion } from 'framer-motion';
import { FaLinkedin } from 'react-icons/fa';

const ContactPage: React.FC = () => {
  return (
    <motion.div
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      className="min-h-screen bg-gray-50"
    >
      <div className="pt-20">
        <section className="py-20 bg-white">
          <div className="container mx-auto px-4">
            <div className="text-center mb-16">
              <h2 className="text-3xl md:text-4xl font-bold mb-4">
                Connect With Us on <span className="text-green-500">Social Media</span>
              </h2>
              <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
              <p className="text-gray-600 max-w-2xl mx-auto">
                Follow us on social media to stay updated with our latest projects, workshops, and robotics innovations.
              </p>
            </div>

            <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
              {/* LinkedIn Section with Icon and Button */}
              <div className="bg-white rounded-xl shadow-lg overflow-hidden flex flex-col items-center justify-center p-8">
                <div className="text-center text-blue-700 mb-4">
                  <FaLinkedin size={100} />
                </div>
                <h3 className="text-xl font-semibold mb-4 text-black">LinkedIn Page</h3>
                <a
                  href="https://www.linkedin.com/company/karthikesh-robotics/"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="px-6 py-2 bg-blue-600 text-white rounded-full hover:bg-blue-700 transition"
                >
                  Visit our LinkedIn Page
                </a>
              </div>

              {/* Instagram Embed */}
              <div className="bg-white rounded-xl shadow-lg overflow-hidden">
                <div className="p-6 bg-black text-white">
                  <h3 className="text-xl font-semibold mb-2">Instagram Feed</h3>
                </div>
                <div className="h-[600px] overflow-hidden">
                  <iframe
                    src="https://www.instagram.com/karthikesh_robotics/embed"
                    width="100%"
                    height="100%"
                    frameBorder="0"
                    scrolling="no"
                    allowTransparency
                    title="Instagram Feed"
                  ></iframe>
                </div>
              </div>

              {/* YouTube Embed */}
              <div className="bg-white rounded-xl shadow-lg overflow-hidden">
                <div className="p-6 bg-black text-white">
                  <h3 className="text-xl font-semibold mb-2">Latest Videos</h3>
                </div>
                <div className="h-[600px] overflow-hidden">
                  <iframe
                    width="100%"
                    height="100%"
                    src="https://www.youtube.com/embed/sxvZ4oxeWt8"
                    frameBorder="0"
                    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                    allowFullScreen
                    title="YouTube Feed"
                  ></iframe>
                </div>
              </div>
            </div>
          </div>
        </section>
      </div>
    </motion.div>
  );
};

export default ContactPage;
