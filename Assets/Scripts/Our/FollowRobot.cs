using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.UI; 

public class FollowRobot : MonoBehaviour
{
    public Transform targetRobot;
    public float followDistance = 5.0f;

    public Transform hand;
    public float leashMaxDistance = 3.0f;
    public Material leashMaterial;
    public float leashWidth = 0.05f;

    private NavMeshAgent agent;
    private Animator animator;
    private LineRenderer leash;

    public Boolean stopFollowing = false;

    public Button toggleFollowButton; 

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        if (targetRobot == null)
        {
            Debug.LogError("FollowRobot: No target robot assigned.");
        }

        if (agent == null)
        {
            Debug.LogError("FollowRobot: No NavMeshAgent component found.");
        }

        if (hand == null)
        {
            Debug.LogError("FollowRobot: No hand transform assigned.");
        }

        leash = gameObject.AddComponent<LineRenderer>();
        leash.material = leashMaterial != null ? leashMaterial : new Material(Shader.Find("Sprites/Default"));
        leash.startWidth = leash.endWidth = leashWidth;
        leash.positionCount = 2;
        leash.enabled = false;

        if (toggleFollowButton != null)
        {
            toggleFollowButton.onClick.AddListener(ToggleFollowing);
        }
        else
        {
            Debug.LogWarning("FollowRobot: Toggle Follow Button is not assigned. Please assign it in the Inspector.");
        }
    }

    void Update()
    {
        if (targetRobot == null || agent == null || hand == null)
            return;


        if (stopFollowing)
        {
            if (agent.hasPath)
            {
                agent.ResetPath();
            }
            animator.SetBool("Walking", false); 
        }
        else 
        {
            float distance = Vector3.Distance(transform.position, targetRobot.position);

            if (distance > followDistance)
            {
                agent.SetDestination(targetRobot.position);
            }
            else
            {
                agent.ResetPath();
            }

            animator.SetBool("Walking", agent.velocity.magnitude != 0);
        }

        float handToRobotDistance = Vector3.Distance(hand.position, targetRobot.position);
        if (!stopFollowing && handToRobotDistance <= leashMaxDistance)
        {
            leash.enabled = true;
            leash.SetPosition(0, hand.position);
            leash.SetPosition(1, targetRobot.position);
        }
        else
        {
            leash.enabled = false;
        }
    }

    public void reattach()
    {
        stopFollowing = false;
    }

    public void ToggleFollowing()
    {
        stopFollowing = !stopFollowing;
        Debug.Log("Following Toggled: " + (!stopFollowing ? "ON" : "OFF"));
    }
}