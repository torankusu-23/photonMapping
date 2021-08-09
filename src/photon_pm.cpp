#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class photon_pm : public Integrator
{
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    photon_pm(const PropertyList &props)
    {
        totalPhotons = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
        emittedPhotons = 0;
    }

    virtual void preprocess(const Scene *scene) override
    {
        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Estimate a default photon radius */
        if (m_photonRadius == 0)
            m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(totalPhotons);

        //晚点用getEmitter试一试，主要不知道enum::EEmitter的值灵不灵
        std::vector<Mesh *> lights;
        for (auto m : scene->getMeshes())
        {
            if (m->isEmitter())
                lights.emplace_back(m);
        }
        int nLights = lights.size();

        //发射+收集光子
        uint32_t storedPhotons = 0;
        while (storedPhotons < totalPhotons)
        {
            Mesh *areaLight = lights[nLights * sampler->next1D()];
            //采样发射光子，这个光子的方向是特殊的，是出射方向，其他都是入射方向（为后续fr作准备）
            Photon emitPhoton = areaLight->getEmitter()->samplePhoton(sampler, areaLight, nLights);

            Ray3f Ray(emitPhoton.getPosition(), emitPhoton.getDirection());
            Intersection its;
            Color3f wait_albedo(1.f);
            uint32_t depth = 0;
            const uint32_t LEAST_DEPTH = 5;

            emittedPhotons++;
            if (scene->rayIntersect(Ray, its))
            {
                while (true)
                {
                    if (its.mesh->getBSDF()->isDiffuse())
                    {
                        Photon p(its.p, -Ray.d, emitPhoton.getPower() * wait_albedo);
                        m_photonMap->push_back(p);
                        storedPhotons++;
                    }

                    BSDFQueryRecord bRec(its.shFrame.toLocal(-Ray.d));
                    Color3f albedo = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                    if (albedo.maxCoeff() == 0.f)
                        break;

                    wait_albedo *= albedo;
                    Ray3f ro(its.p, its.shFrame.toWorld(bRec.wo));
                    Intersection next_its;

                    if (!scene->rayIntersect(ro, next_its))
                    {
                        break;
                    }
                    Ray.o = ro.o;
                    Ray.d = ro.d;
                    its = next_its;

                    if (depth < LEAST_DEPTH)
                    {
                        depth++;
                    }
                    else
                    {
                        //RR
                        float q = wait_albedo.maxCoeff();
                        if (sampler->next1D() > q)
                            break;
                        wait_albedo /= q;
                    }
                }
            }
        }

        /* Build the photon map */
        m_photonMap->build();
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) override
    {
        Intersection its;
        Color3f result(0.f);
        //check if the ray intersects the scene
        if (!scene->rayIntersect(ray, its))
        {
            return result;
        }

        Color3f wait_albedo(1.f);
        Ray3f Ray(ray);
        Intersection x(its);
        const uint32_t LEAST_DEPTH = 5;
        uint32_t depth = 0;

        while (true)
        {
            //get the radiance of hitten object
            if (x.mesh->isEmitter())
            {
                EmitterQueryRecord eRec(Ray.o, x.p, x.shFrame.n);
                result += wait_albedo * x.mesh->getEmitter()->eval(eRec);
            }

            if (x.mesh->getBSDF()->isDiffuse())
            {
                std::vector<uint32_t> local_photon;
                m_photonMap->search(x.p, m_photonRadius, local_photon);
                int len = local_photon.size();

                Color3f local_power(0.f);
                for (int i = 0; i < len; ++i)
                {
                    Photon &photon = (*m_photonMap)[local_photon[i]];
                    BSDFQueryRecord bRec(x.shFrame.toLocal(photon.getDirection()), x.shFrame.toLocal(-Ray.d), ESolidAngle);
                    Color3f fr = x.mesh->getBSDF()->eval(bRec);
                    local_power += wait_albedo * fr * photon.getPower() / (M_PI * m_photonRadius * m_photonRadius);
                }
                if (local_power.maxCoeff() > 0.f)
                    result += local_power / emittedPhotons;

                break;
            }

            //mirror or dielectric
            BSDFQueryRecord bRec(x.shFrame.toLocal(-Ray.d));
            Color3f albedo = x.mesh->getBSDF()->sample(bRec, sampler->next2D());
            if (albedo.maxCoeff() == 0.f)
                break;
            wait_albedo *= albedo;

            Intersection next_x;
            Ray3f ro(x.p, x.shFrame.toWorld(bRec.wo));

            //ray escapes the scene
            if (!scene->rayIntersect(ro, next_x))
                break;

            x = next_x;
            Ray.o = ro.o;
            Ray.d = ro.d;

            if (depth < LEAST_DEPTH)
            {
                depth++;
            }
            else
            {
                //stop critirium russian roulette
                float q = std::min(0.99f, wait_albedo.maxCoeff());
                if (q < sampler->next1D())
                    break;
                wait_albedo /= q;
            }
        }

        return result;
    }

    virtual std::string toString() const override
    {
        return tfm::format(
            "PhotonMapper[\n"
            "  totalPhotons = %i,\n"
            "  photonRadius = %f\n"
            "  emittedPhotons = %f\n"
            "]",
            totalPhotons,
            m_photonRadius,
            emittedPhotons);
    }

private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */
    uint32_t totalPhotons;
    uint32_t emittedPhotons;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(photon_pm, "photon_pm");
NORI_NAMESPACE_END