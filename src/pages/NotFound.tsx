import React from 'react';
import { motion } from 'framer-motion';
import { Link } from 'react-router-dom';
import { Home, RefreshCw } from 'lucide-react';

const NotFound: React.FC = () => {
  return (
    <div className="min-h-screen bg-black text-white flex items-center justify-center p-4">
      <div className="max-w-2xl w-full text-center">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
        >
          <div className="relative mb-8">
            <motion.div
              animate={{
                rotate: [0, 360],
                scale: [1, 1.2, 1],
              }}
              transition={{
                duration: 3,
                repeat: Infinity,
                repeatType: "reverse",
              }}
              className="w-32 h-32 mx-auto"
            >
              <RefreshCw className="w-full h-full text-green-500" />
            </motion.div>
            <motion.div
              animate={{
                opacity: [0, 1, 0],
              }}
              transition={{
                duration: 2,
                repeat: Infinity,
              }}
              className="absolute inset-0 bg-green-500/20 rounded-full blur-xl"
            />
          </div>

          <h1 className="text-6xl font-bold mb-4">
            4
            <motion.span
              animate={{
                rotate: [0, 10, -10, 0],
              }}
              transition={{
                duration: 2,
                repeat: Infinity,
              }}
              className="inline-block text-green-500"
            >
              0
            </motion.span>
            4
          </h1>

          <p className="text-2xl mb-8 text-gray-400">
            Looks like our robots encountered an unexpected error!
          </p>

          <div className="flex justify-center gap-4">
            <Link
              to="/"
              className="inline-flex items-center gap-2 bg-green-500 hover:bg-green-600 text-black font-semibold py-3 px-6 rounded-lg transition-colors"
            >
              <Home className="h-5 w-5" />
              Return Home
            </Link>
          </div>
        </motion.div>

        <motion.div
          className="mt-12 p-6 bg-gray-900 rounded-xl"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.3 }}
        >
          <div className="font-mono text-sm text-gray-400">
            <p className="mb-2">{'> ERROR_LOG:'}</p>
            <p className="text-green-500">{'> SYSTEM_MALFUNCTION_DETECTED'}</p>
            <p>{'> INITIATING_RECOVERY_SEQUENCE...'}</p>
            <p>{'> REROUTING_TO_HOMEPAGE...'}</p>
          </div>
        </motion.div>
      </div>
    </div>
  );
};

export default NotFound;