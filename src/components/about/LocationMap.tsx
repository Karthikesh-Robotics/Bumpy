import React from 'react';
import { MapPin, Phone, Mail } from 'lucide-react';

const LocationMap: React.FC = () => {
  return (
    <section className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-5xl font-bold text-black">
            Find Us
          </h2>
          <p className="text-gray-600 mt-4 max-w-2xl mx-auto">
            Visit our headquarters or get in touch with us directly.
          </p>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </div>

        <div className="grid md:grid-cols-2 gap-12 items-center">
          {/* Map */}
          <div className="h-[400px] rounded-xl overflow-hidden shadow-lg">
            <iframe
              src="https://www.google.com/maps/embed?pb=!1m18!1m12!1m3!1d3887.970970819419!2d80.12826067574757!3d12.973708514830724!2m3!1f0!2f0!3f0!3m2!1i1024!2i768!4f13.1!3m3!1m2!1s0x3a525fc381bde825%3A0x641e33c1866b1cc8!2sKarthikesh%20Robotics%20Private%20Limited!5e0!3m2!1sen!2sin!4v1748016910878!5m2!1sen!2sin"
              width="100%"
              height="100%"
              style={{ border: 0 }}
              allowFullScreen
              loading="lazy"
              referrerPolicy="no-referrer-when-downgrade"
              title="Karthikesh Robotics Location"
            ></iframe>
          </div>

          {/* Contact information */}
          <div className="p-8 bg-black text-white rounded-xl shadow-lg">
            <h3 className="text-2xl font-bold mb-8">Contact Information</h3>

            <div className="space-y-6">
              <div className="flex items-start">
                <div className="bg-green-500 p-3 rounded-full mr-4">
                  <MapPin className="h-6 w-6 text-black" />
                </div>
                <div>
                  <h4 className="text-lg font-bold mb-2">Our Location</h4>
                  <p className="text-gray-300">
                    Karthikesh Robotics Private Limited<br />
                    Pammal<br />
                    Chennai, Tamil Nadu 600075<br />
                    India
                  </p>
                </div>
              </div>

              <div className="flex items-start">
                <div className="bg-green-500 p-3 rounded-full mr-4">
                  <Phone className="h-6 w-6 text-black" />
                </div>
                <div>
                  <h4 className="text-lg font-bold mb-2">Phone</h4>
                  <p className="text-gray-300">
                    +91 86083 54107
                  </p>
                </div>
              </div>

              <div className="flex items-start">
                <div className="bg-green-500 p-3 rounded-full mr-4">
                  <Mail className="h-6 w-6 text-black" />
                </div>
                <div>
                  <h4 className="text-lg font-bold mb-2">Email</h4>
                  <p className="text-gray-300">
                    karthikeshrobotics@gmail.com<br />
                    support@karthikeshrobotics.in
                  </p>
                </div>
              </div>
            </div>

            <a
              href="https://maps.google.com/?q=Karthikesh Robotics Private Limited"
              target="_blank"
              rel="noopener noreferrer"
              className="mt-8 inline-block bg-green-500 hover:bg-green-600 text-black font-semibold py-3 px-8 rounded-md transition-all duration-300"
            >
              Get Directions
            </a>
          </div>
        </div>
      </div>
    </section>
  );
};

export default LocationMap;
