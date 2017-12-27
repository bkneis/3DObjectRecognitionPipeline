
namespace pipeline {

    class RecognitionPipeline {

    public:

        virtual void visualize() = 0;

        virtual void describe() = 0;

        virtual void classify() = 0;

        static RecognitionPipeline*
        create(Config* conf)
        {
            if (!conf->getFeatureDescriptorStrategy().compare(FPFH)) {
                return new Hist308RecognitionPipeline(conf);
            }

            return new Hist33RecognitionPipeline(conf);
        }

        void
        estimateSurfaceNormals()
        {

        }

        void
        run()
        {
            // Perform segmentation and remove background
            input = preprocessor::removeOutliers(input, 0.3, 300);

            auto t1 = std::chrono::high_resolution_clock::now();

            estimateSurfaceNormals();
            describe();
            classify();

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            cout << "Pipeline took " << duration << " micro seconds" << endl;
        }

    protected:

        PointCloudPtr input;
        PointCloudPtr normals;

    };

    class Hist33RecognitionPipeline: public RecognitionPipeline {

        Hist33RecognitionPipeline(Config* conf)
        {
            // Initialise classifier and populate database
        }

        void
        visualize()
        {

        }

        void
        describe()
        {

        }

        void
        classify()
        {

        }

    };

    class Hist308RecognitionPipeline: public RecognitionPipeline {

        Hist308RecognitionPipeline(Config* conf)
        {
            // Initialise classifier and populate database
        }

        void
        visualize()
        {

        }

        void
        describe()
        {

        }

        void
        classify()
        {

        }

    };
}
