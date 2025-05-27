import React from 'react';
import { Notebook as Robot, Facebook, Twitter, Linkedin, Instagram, ArrowUp, Youtube, Mail } from 'lucide-react';
import { Link } from 'react-router-dom';

const Footer: React.FC = () => {
  const scrollToTop = () => {
    window.scrollTo({
      top: 0,
      behavior: 'smooth',
    });
  };

  return (
    <footer className="bg-black text-white pt-16 pb-8">
      <div className="container mx-auto px-4">
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 mb-12">
          <div>
            <div className="flex items-center mb-6">
              <Robot className="h-8 w-8 text-green-500 mr-2" />
              <span className="text-white font-semibold text-xl">
                Karthikesh Robotics
              </span>
            </div>
            <p className="text-gray-400 mb-6">
              Pioneering the future of autonomous robotics through innovation, education, and implementation.
            </p>
            <div className="flex space-x-4">
              <a
                href="https://www.linkedin.com/company/karthikesh-robotics"
                target="_blank"
                rel="noopener noreferrer"
                className="bg-gray-800 hover:bg-green-500 text-white hover:text-black w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300"
              >
                <Linkedin className="h-5 w-5" />
              </a>
              <a
                href="https://www.instagram.com/karthikesh_robotics/"
                target="_blank"
                rel="noopener noreferrer"
                className="bg-gray-800 hover:bg-green-500 text-white hover:text-black w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300"
              >
                <Instagram className="h-5 w-5" />
              </a>
              <a
                href="https://www.youtube.com/@KarthiKeshRobotics"
                target="_blank"
                rel="noopener noreferrer"
                className="bg-gray-800 hover:bg-green-500 text-white hover:text-black w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300"
              >
                <Youtube className="h-5 w-5" />
              </a>
              <a
                href="mailto:karthikeshrobotics@gmail.com"
                className="bg-gray-800 hover:bg-green-500 text-white hover:text-black w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300"
              >
                <Mail className="h-5 w-5" />
              </a>
            </div>
          </div>

          <div>
            <h3 className="text-xl font-semibold mb-6">Quick Links</h3>
            <ul className="space-y-3">
              {['Home', 'About', 'Services', 'Products', 'Contact'].map((item) => (
                <li key={item}>
                  <Link
                    to={`/${item.toLowerCase() === 'home' ? '' : item.toLowerCase()}`}
                    className="text-gray-400 hover:text-green-400 transition-colors duration-300"
                  >
                    {item}
                  </Link>
                </li>
              ))}
            </ul>
          </div>

          <div>
            <h3 className="text-xl font-semibold mb-6">Our Services</h3>
            <ul className="space-y-3">
              {[
                'Autonomous Mobile Robots',
                'ROS2 Mentoring',
                'Physical AI Solutions',
                'Educational Workshops',
                'Robotics Lab Setup',
                'Custom Robotics Development'
              ].map((item) => (
                <li key={item}>
                  <Link
                    to="/services"
                    className="text-gray-400 hover:text-green-400 transition-colors duration-300"
                  >
                    {item}
                  </Link>
                </li>
              ))}
            </ul>
          </div>

          <div>
            <h3 className="text-xl font-semibold mb-6">Newsletter</h3>
            <p className="text-gray-400 mb-4">
              Subscribe to our newsletter for the latest updates on robotics innovations and workshops.
            </p>
            <form className="flex mb-4">
              <input
                type="email"
                placeholder="Your email"
                className="bg-gray-800 border border-gray-700 rounded-l-lg px-4 py-2 w-full focus:outline-none focus:ring-2 focus:ring-green-500 focus:border-transparent"
              />
              <button
                type="submit"
                className="bg-green-500 hover:bg-green-600 text-black font-medium px-4 rounded-r-lg transition-colors duration-300"
              >
                Subscribe
              </button>
            </form>
          </div>
        </div>

        <div className="border-t border-gray-800 pt-8 flex flex-col md:flex-row justify-between items-center">
          <p className="text-gray-400 text-center md:text-left mb-4 md:mb-0">
            &copy; {new Date().getFullYear()} Karthikesh Robotics. All rights reserved.
          </p>
          <button
            onClick={scrollToTop}
            className="bg-green-500 hover:bg-green-600 text-black w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300"
          >
            <ArrowUp className="h-5 w-5" />
          </button>
        </div>
      </div>
    </footer>
  );
};

export default Footer;