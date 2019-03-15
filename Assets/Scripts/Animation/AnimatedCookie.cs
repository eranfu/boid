using UnityEngine;

namespace Animation
{
    public class AnimatedCookie : MonoBehaviour
    {
        [SerializeField] private Texture[] cookies = null;
        [SerializeField] private float framesPerSecond = 15;
        private Light _light;
        private int _index = 0;
        private float _nextCookieTime = 0;

        private void Awake()
        {
            _light = GetComponent<Light>();
        }

        private void OnEnable()
        {
            _nextCookieTime = Time.time;
        }

        private void Update()
        {
            float time = Time.time;
            if (time >= _nextCookieTime)
            {
                _light.cookie = cookies[_index];
                ++_index;
                if (_index == cookies.Length)
                {
                    _index = 0;
                }

                float step = 1 / framesPerSecond;
                _nextCookieTime += step;
                if (_nextCookieTime <= time)
                {
                    _nextCookieTime = time + step;
                }
            }
        }
    }
}