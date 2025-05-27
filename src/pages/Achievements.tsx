import React, { useState, useEffect } from 'react';
import { achievements } from '../data/achievements';
import AchievementCard from '../components/AchievementCard';
import YearFilter from '../components/YearFilter';
import CategoryFilter from '../components/CategoryFilter';
import { Trophy } from 'lucide-react';
import { motion } from 'framer-motion';

const Achievements: React.FC = () => {
  const [selectedYear, setSelectedYear] = useState<number | null>(null);
  const [selectedCategory, setSelectedCategory] = useState<string | null>(null);
  const [filteredAchievements, setFilteredAchievements] = useState(achievements);
  const [visibleCount, setVisibleCount] = useState(6);
  const [searchQuery, setSearchQuery] = useState('');

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

  // Extract unique years and categories
  const years = [...new Set(achievements.map(a => a.year))].sort().reverse();
  const uniqueCategories = [...new Set(achievements.map(a => a.category))].sort();

  // Apply filters
  useEffect(() => {
    let filtered = achievements;

    if (selectedYear !== null) {
      filtered = filtered.filter(a => a.year === selectedYear);
    }

    if (selectedCategory !== null) {
      filtered = filtered.filter(a => a.category === selectedCategory);
    }

    if (searchQuery.trim() !== '') {
      const query = searchQuery.toLowerCase();
      filtered = filtered.filter(a =>
        a.title.toLowerCase().includes(query) ||
        a.location.toLowerCase().includes(query)
      );
    }

    filtered = [...filtered].sort((a, b) => {
      const dateA = new Date(a.date.replace(/(\d+)(st|nd|rd|th)/, '$1'));
      const dateB = new Date(b.date.replace(/(\d+)(st|nd|rd|th)/, '$1'));
      return dateB.getTime() - dateA.getTime();
    });

    setFilteredAchievements(filtered);
  }, [selectedYear, selectedCategory, searchQuery]);

  const loadMore = () => {
    setVisibleCount(prevCount => Math.min(prevCount + 6, filteredAchievements.length));
  };

  return (
    <motion.section 
      className="py-20 bg-gray-50"
      variants={containerVariants}
      initial="hidden"
      animate="visible"
    >
      <div className="container mx-auto px-4">
        <motion.div 
          className="text-center mb-16"
          variants={itemVariants}
        >
          <h2 className="text-3xl md:text-4xl font-bold mb-4 text-gray-900">
            Our <span className="text-green-500">Achievements</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Explore our past workshops and events, showcasing our commitment to robotics education and innovation.
          </p>
        </motion.div>

        <motion.div 
          className="mb-8"
          variants={itemVariants}
        >
          <div className="relative max-w-md mx-auto mb-8">
            <input
              type="text"
              placeholder="Search workshops by title or location..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="w-full px-10 py-3 rounded-full border border-gray-200 focus:outline-none focus:border-green-500 transition-colors"
            />
            <Trophy className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400 h-5 w-5" />
            {searchQuery && (
              <button
                onClick={() => setSearchQuery('')}
                className="absolute right-3 top-1/2 transform -translate-y-1/2 text-gray-400 hover:text-gray-600"
              >
                <span className="sr-only">Clear search</span>
                ×
              </button>
            )}
          </div>

          <motion.div 
            className="flex flex-col sm:flex-row justify-center items-center gap-4"
            variants={itemVariants}
          >
            <YearFilter
              years={years}
              selectedYear={selectedYear}
              onSelectYear={setSelectedYear}
            />
            <CategoryFilter
              categories={uniqueCategories}
              selectedCategory={selectedCategory}
              onSelectCategory={setSelectedCategory}
            />
          </motion.div>
        </motion.div>

        {filteredAchievements.length === 0 ? (
          <motion.div 
            className="text-center py-16"
            variants={itemVariants}
          >
            <Trophy className="h-16 w-16 text-gray-300 mx-auto mb-4" />
            <h3 className="text-xl font-medium text-gray-900 mb-2">No workshops found</h3>
            <p className="text-gray-600">Try adjusting your filters or search query</p>
          </motion.div>
        ) : (
          <motion.div 
            className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8"
            variants={containerVariants}
          >
            {filteredAchievements.slice(0, visibleCount).map((achievement, index) => (
              <motion.div
                key={achievement.id}
                variants={itemVariants}
              >
                <AchievementCard
                  id={achievement.id}
                  date={achievement.date}
                  title={achievement.title}
                  location={achievement.location}
                  category={achievement.category}
                  images={achievement.images}
                  expanded={index === 0}
                />
              </motion.div>
            ))}
          </motion.div>
        )}

        {visibleCount < filteredAchievements.length && (
          <motion.div 
            className="text-center mt-12"
            variants={itemVariants}
          >
            <button
              onClick={loadMore}
              className="bg-green-500 hover:bg-green-600 text-white font-semibold py-3 px-8 rounded-full transition-all duration-300 transform hover:scale-105"
            >
              Load More
            </button>
          </motion.div>
        )}
      </div>
    </motion.section>
  );
};

export default Achievements;