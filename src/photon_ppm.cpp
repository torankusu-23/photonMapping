#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class photon_ppm : public Integrator
{
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    photon_ppm(const PropertyList &props)
    {
        totalPhotons = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
        emittedPhotons = 0;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) override
    {
    
        return 0.f;
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

NORI_REGISTER_CLASS(photon_ppm, "photon_ppm");
NORI_NAMESPACE_END