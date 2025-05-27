import React from 'react';
import { Calendar, ArrowRight } from 'lucide-react';

interface NewsItem {
  title: string;
  date: string;
  excerpt: string;
  image: string;
  url: string;
}

const LatestNews: React.FC = () => {
  const newsItems: NewsItem[] = [
    {
      title: "KKR Launches New Educational Robotics Program",
      date: "June 15, 2024",
      excerpt: "Karthikesh Robotics announces a new program designed to bring robotics education to underserved schools across the region.",
      image: "https://images.pexels.com/photos/8566526/pexels-photo-8566526.jpeg",
      url: "#"
    },
    {
      title: "Partnership with Leading University Announced",
      date: "May 28, 2024",
      excerpt: "KKR forms strategic partnership with prestigious university to advance robotics research and development.",
      image: "https://images.pexels.com/photos/2599244/pexels-photo-2599244.jpeg",
      url: "#"
    },
    {
      title: "New Autonomous Navigation System Unveiled",
      date: "April 10, 2024",
      excerpt: "Our team has developed a breakthrough in autonomous navigation technology that improves efficiency by 40%.",
      image: "https://images.pexels.com/photos/39644/robonaut-machines-dexterous-humanoid-39644.jpeg",
      url: "#"
    }
  ];

  return (
    <section className="py-20 bg-gray-50">
      <div className="container mx-auto px-4">
        <div className="flex justify-between items-center mb-16">
          <div>
            <h2 className="text-3xl md:text-5xl font-bold text-black">
              Latest News
            </h2>
            <div className="w-24 h-1 bg-green-500 mt-6"></div>
          </div>
          <a 
            href="#" 
            className="hidden md:flex items-center text-green-500 hover:text-green-600 transition-colors duration-300"
          >
            View All News
            <ArrowRight className="ml-2 h-4 w-4" />
          </a>
        </div>
        
        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
          {newsItems.map((news, index) => (
            <div 
              key={index}
              className="bg-white rounded-xl overflow-hidden shadow-lg transition-all duration-300 hover:shadow-2xl hover:-translate-y-2"
            >
              <div className="h-48 overflow-hidden">
                <img 
                  src={news.image} 
                  alt={news.title}
                  className="w-full h-full object-cover object-center transition-transform duration-500 hover:scale-110"
                />
              </div>
              
              <div className="p-6">
                <div className="flex items-center text-gray-500 mb-3">
                  <Calendar className="h-4 w-4 mr-2" />
                  <span className="text-sm">{news.date}</span>
                </div>
                
                <h3 className="text-xl font-bold mb-3 line-clamp-2">{news.title}</h3>
                <p className="text-gray-600 mb-4 line-clamp-3">{news.excerpt}</p>
                
                <a 
                  href={news.url}
                  className="inline-flex items-center text-green-500 hover:text-green-600 transition-colors duration-300"
                >
                  Read More
                  <ArrowRight className="ml-2 h-4 w-4" />
                </a>
              </div>
            </div>
          ))}
        </div>
        
        <div className="mt-8 text-center md:hidden">
          <a 
            href="#" 
            className="inline-flex items-center text-green-500 hover:text-green-600 transition-colors duration-300"
          >
            View All News
            <ArrowRight className="ml-2 h-4 w-4" />
          </a>
        </div>
      </div>
    </section>
  );
};

export default LatestNews;