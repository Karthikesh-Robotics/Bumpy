import React from 'react';
import { Calendar, Clock, Globe, ArrowRight } from 'lucide-react';

const workshops = [
  {
    title: 'ROS 2: Creating URDF, ROS2 CONTROL & SLAM',
    date: 'May 31, 2025',
    time: '3:30 PM - 6:00 PM',
    mode: 'Online',
    description:
      'Dive into URDF creation, setting up ROS2 Control, and mastering SLAM for autonomous robotics applications.',
    level: 'Intermediate',
    price: '₹99',
    image: '/assets/workshop.png', // Make sure this file is in your public folder
    registerLink:
      'https://unstop.com/p/ros-2-creating-urdf-ros2-control-slam-karthikesh-robotics-private-limited-1482113',
  },
];

const Workshops: React.FC = () => {
  return (
    <section id="workshops" className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Upcoming <span className="text-green-500">Workshops</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Join our hands-on workshops and master the latest in robotics technology.
          </p>
        </div>

        <div className="flex justify-center">
          <div className="max-w-md w-full">
            {workshops.map((workshop, index) => (
              <div
                key={workshop.title}
                className="bg-white rounded-lg shadow-lg overflow-hidden group hover:shadow-2xl transition-all duration-300 transform hover:-translate-y-2"
                style={{ animationDelay: `${index * 0.2}s` }}
              >
                <div className="aspect-video overflow-hidden">
                  <img
                    src={workshop.image}
                    alt={workshop.title}
                    className="w-full h-full object-cover transition-transform duration-500 group-hover:scale-110"
                  />
                </div>

                <div className="p-6">
                  <div className="flex items-center justify-between mb-4">
                    <span className="px-3 py-1 bg-green-100 text-green-500 rounded-full text-sm font-medium">
                      {workshop.level}
                    </span>
                    <span className="text-xl font-bold text-green-500">
                      {workshop.price}
                    </span>
                  </div>

                  <h3 className="text-xl font-semibold mb-4 group-hover:text-green-500 transition-colors">
                    {workshop.title}
                  </h3>

                  <p className="text-gray-600 mb-6">{workshop.description}</p>

                  <div className="space-y-3 mb-6">
                    <div className="flex items-center gap-3 text-gray-600">
                      <Calendar className="h-5 w-5 text-green-500" />
                      <span>{workshop.date}</span>
                    </div>
                    <div className="flex items-center gap-3 text-gray-600">
                      <Clock className="h-5 w-5 text-green-500" />
                      <span>{workshop.time}</span>
                    </div>
                    <div className="flex items-center gap-3 text-gray-600">
                      <Globe className="h-5 w-5 text-green-500" />
                      <span>{workshop.mode}</span>
                    </div>
                  </div>

                  <a
                    href={workshop.registerLink}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="w-full bg-gray-900 hover:bg-green-500 text-white font-semibold py-3 px-6 rounded-md transition-all duration-300 flex items-center justify-center gap-2 group"
                  >
                    Register Now
                    <ArrowRight className="h-5 w-5 transform group-hover:translate-x-1 transition-transform" />
                  </a>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
};

export default Workshops;
