import React from 'react';
import { ArrowRight } from 'lucide-react';

const blogPosts = [
  {
    id: 1,
    title: 'The Future of Autonomous Navigation in Mobile Robots',
    excerpt: 'Exploring the latest advancements in autonomous navigation systems and their real-world applications.',
    image: 'https://images.pexels.com/photos/2599244/pexels-photo-2599244.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    date: 'May 15, 2025',
    author: 'Dr. Karthik Raman',
  },
  {
    id: 2,
    title: 'Implementing ROS2 in Educational Environments',
    excerpt: 'A comprehensive guide to setting up ROS2 frameworks for educational purposes in universities and research labs.',
    image: 'https://images.pexels.com/photos/8566543/pexels-photo-8566543.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    date: 'April 22, 2025',
    author: 'Priya Sharma',
  },
  {
    id: 3,
    title: 'Physical AI: Bridging the Gap Between Digital and Physical Worlds',
    excerpt: 'How Physical AI is revolutionizing robotics by enabling more intuitive interactions with the environment.',
    image: 'https://images.pexels.com/photos/1476321/pexels-photo-1476321.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=2',
    date: 'March 10, 2025',
    author: 'Dr. Rahul Mehta',
  },
];

const BlogPreview: React.FC = () => {
  return (
    <section id="blog" className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Our <span className="text-green-500">Blog</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Insights and updates from the world of robotics, AI, and educational technology.
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
          {blogPosts.map((post) => (
            <div key={post.id} className="bg-white rounded-lg shadow-md overflow-hidden group hover:shadow-xl transition-all duration-300">
              <div className="aspect-video overflow-hidden">
                <img
                  src={post.image}
                  alt={post.title}
                  className="w-full h-full object-cover transition-transform duration-500 group-hover:scale-110"
                />
              </div>
              
              <div className="p-6">
                <div className="flex items-center justify-between text-sm text-gray-500 mb-3">
                  <span>{post.date}</span>
                  <span>By {post.author}</span>
                </div>
                
                <h3 className="text-xl font-semibold mb-3 group-hover:text-green-500 transition-colors duration-300">
                  {post.title}
                </h3>
                
                <p className="text-gray-600 mb-4">
                  {post.excerpt}
                </p>
                
                <a
                  href="#"
                  className="inline-flex items-center text-green-500 hover:text-green-700 transition-colors duration-300 font-medium"
                >
                  Read More
                  <ArrowRight className="h-4 w-4 ml-2 transform group-hover:translate-x-1 transition-transform duration-300" />
                </a>
              </div>
            </div>
          ))}
        </div>

        <div className="text-center mt-12">
          <a
            href="#"
            className="inline-block bg-green-500 hover:bg-green-600 text-black font-semibold py-3 px-8 rounded-md transition-all duration-300"
          >
            View All Articles
          </a>
        </div>
      </div>
    </section>
  );
};

export default BlogPreview;