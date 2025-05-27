import React, { useState } from 'react';
import PortfolioItem from './PortfolioItem';

const categories = ['All', 'Robots', 'Workshops', 'Lab Setup', 'Research'];

const portfolioItems = [
  {
    id: 1,
    title: 'Autonomous Navigation Robot',
    category: 'Robots',
    image: 'https://images.pexels.com/photos/2599244/pexels-photo-2599244.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Custom-built autonomous robot with advanced navigation capabilities for industrial applications.',
  },
  {
    id: 2,
    title: 'ROS2 Workshop at Tech University',
    category: 'Workshops',
    image: 'https://images.pexels.com/photos/8566429/pexels-photo-8566429.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Intensive workshop on ROS2 implementation and programming for robotics students.',
  },
  {
    id: 3,
    title: 'Advanced Robotics Lab Setup',
    category: 'Lab Setup',
    image: 'https://images.pexels.com/photos/8566543/pexels-photo-8566543.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Complete robotics laboratory setup for a leading engineering college with training and support.',
  },
  {
    id: 4,
    title: 'Physical AI Research Project',
    category: 'Research',
    image: 'https://images.pexels.com/photos/355948/pexels-photo-355948.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Collaborative research on implementing physical AI systems for environmental monitoring.',
  },
  {
    id: 5,
    title: 'Educational Robot Series',
    category: 'Robots',
    image: 'https://images.pexels.com/photos/2599243/pexels-photo-2599243.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Series of educational robots designed specifically for classroom learning and experimentation.',
  },
  {
    id: 6,
    title: 'Robotics Summer Camp',
    category: 'Workshops',
    image: 'https://images.pexels.com/photos/8566473/pexels-photo-8566473.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    description: 'Two-week intensive summer camp introducing high school students to robotics principles.',
  },
];

const Portfolio: React.FC = () => {
  const [activeCategory, setActiveCategory] = useState('All');

  const filteredItems = activeCategory === 'All' 
    ? portfolioItems 
    : portfolioItems.filter(item => item.category === activeCategory);

  return (
    <section id="portfolio" className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Our <span className="text-green-500">Projects</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Explore our innovative robotics projects, workshops, and lab setups across educational institutions.
          </p>
        </div>

        <div className="flex flex-wrap justify-center gap-4 mb-12">
          {categories.map((category) => (
            <button
              key={category}
              onClick={() => setActiveCategory(category)}
              className={`px-6 py-2 rounded-full transition-all duration-300 ${
                activeCategory === category
                  ? 'bg-green-500 text-white'
                  : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
              }`}
            >
              {category}
            </button>
          ))}
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8">
          {filteredItems.map((item) => (
            <PortfolioItem
              key={item.id}
              title={item.title}
              category={item.category}
              image={item.image}
              description={item.description}
            />
          ))}
        </div>
      </div>
    </section>
  );
};

export default Portfolio;