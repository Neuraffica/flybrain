"""flybrain-models: models built on flybrain-data circuit packs.

>>> from flybrain_models.compass import RingAttractor, GyroIntegrator
>>> fly = RingAttractor.from_pack("central_complex")          # connectome compass
>>> heading = fly.step(angular_velocity_rad_s=0.5, dt=0.02)
"""
